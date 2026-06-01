// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// Jupiter Voice Node
// Audio capture is offloaded to the Raspberry Pi 5 sub-node (see
// jupiter_audio_capture): this node SUBSCRIBES to /audio/mic_raw (16 kHz mono
// S16 frames over the wired link), accumulates them into analysis windows, and
// feeds whisper.cpp ASR → /voice/raw_text. /voice/response_text → piper TTS.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "whisper.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

float compute_rms_energy(const std::vector<int16_t>& samples) {
    if (samples.empty()) return 0.0f;
    double sum_sq = 0.0;
    for (const int16_t s : samples) {
        sum_sq += static_cast<double>(s) * s;
    }
    return static_cast<float>(std::sqrt(sum_sq / samples.size()));
}

std::vector<float> pcm16_to_float32(const std::vector<int16_t>& pcm) {
    std::vector<float> out;
    out.reserve(pcm.size());
    constexpr float kScale = 1.0f / 32768.0f;
    for (const int16_t s : pcm) {
        out.push_back(static_cast<float>(s) * kScale);
    }
    return out;
}

// Returns true if the ASR result is a Whisper noise label rather than speech.
// Whisper emits these for non-speech sounds: [Music], [Barking], (applause), *beep*, etc.
bool is_noise_label(const std::string& text) {
    if (text.empty()) return false;
    const char first = text.front();
    const char last  = text.back();
    if (first == '[' && last == ']') return true;
    if (first == '(' && last == ')') return true;
    if (first == '*' && last == '*') return true;
    return false;
}

// Remove shell-sensitive characters before passing text to piper subprocess
std::string sanitize_for_shell(std::string text) {
    for (char& c : text) {
        if (c == '\'' || c == '"' || c == '`' || c == '\\' || c == '$') {
            c = ' ';
        }
    }
    return text;
}

std::string default_path(const std::string& rel) {
    const char* home = std::getenv("HOME");
    return std::string(home ? home : "/home/jupiter") + "/" + rel;
}

} // namespace


class JupiterVoice : public rclcpp::Node {
public:
    explicit JupiterVoice() : Node("jupiter_voice") {
        declare_parameters();
        init_whisper();

        text_pub_     = create_publisher<std_msgs::msg::String>("/voice/raw_text", 10);
        tts_done_pub_ = create_publisher<std_msgs::msg::Empty>("/voice/tts_done", 1);

        response_sub_ = create_subscription<std_msgs::msg::String>(
            "/voice/response_text", 10,
            std::bind(&JupiterVoice::response_callback, this, std::placeholders::_1));

        // Brain raises this while awaiting a name or yes/no confirmation during
        // guest registration — relaxes the 2-word minimum so single-word answers
        // ("Logan", "yes", "no") are not dropped as hallucinations.
        expecting_name_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/jupiter/expecting_name", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                expecting_name_.store(msg->data);
                RCLCPP_INFO(get_logger(), "[REG] expecting_name = %s",
                            msg->data ? "true" : "false");
            });

        // Audio arrives from the Pi 5 capture node as continuous 16 kHz mono S16
        // frames. Best-Effort + shallow history to match the publisher and keep
        // latency low on the wired link. The callback just appends to accum_.
        rclcpp::QoS audio_qos(rclcpp::KeepLast(50));
        audio_qos.best_effort();
        audio_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
            "/audio/mic_raw", audio_qos,
            std::bind(&JupiterVoice::audio_callback, this, std::placeholders::_1));

        audio_thread_ = std::thread(&JupiterVoice::audio_loop, this);

        RCLCPP_INFO(get_logger(), "Jupiter Voice online — /audio/mic_raw subscriber + whisper.cpp ASR + piper TTS");
        RCLCPP_INFO(get_logger(), "Window %ds @ %dHz  |  VAD SNR ratio %.1f  |  model: %s",
            record_seconds_, sample_rate_, vad_snr_ratio_, whisper_model_.c_str());
    }

    ~JupiterVoice() {
        running_ = false;
        const pid_t tts = tts_pid_.load();
        if (tts > 0) ::kill(tts, SIGTERM);
        if (audio_thread_.joinable()) {
            audio_thread_.join();
        }
        if (whisper_ctx_) {
            whisper_free(whisper_ctx_);
        }
    }

private:
    // ── Parameters ────────────────────────────────────────────────────────────

    void declare_parameters() {
        declare_parameter("whisper_model",
            default_path("jupitercpp_ws/whisper.cpp/models/ggml-medium.en.bin"));
        declare_parameter("piper_binary",
            default_path("jupitercpp_ws/piper_tts/piper/piper"));
        declare_parameter("piper_model",
            default_path("jupitercpp_ws/piper_tts/piper/en_US-lessac-high.onnx"));
        declare_parameter("sample_rate",        16000);
        declare_parameter("record_seconds",     5);
        declare_parameter("energy_threshold",   300.0);
        declare_parameter("whisper_threads",    4);
        declare_parameter("whisper_language",   std::string("en"));
        declare_parameter("response_timeout_s", 90);
        declare_parameter("asr_cooldown_s",     0);
        // Minimum peak-to-trough RMS ratio across 100ms windows to classify as speech.
        // HVAC steady noise ≈ 1.05-1.2; speech (starts/stops/pauses) ≈ 2.0-10.0.
        declare_parameter("vad_snr_ratio",      1.7);
        // TTS output sink — route playback to HDMI (7-inch display speaker) so it
        // does NOT play through the ReSpeaker. Playing TTS through the ReSpeaker's
        // own output wedges its USB mic capture into silence (energy ~21). Keeping
        // the ReSpeaker capture-only avoids the full-duplex wedge. Empty = default sink.
        declare_parameter("tts_sink",
            std::string("alsa_output.platform-88090b0000.hda.hdmi-stereo"));

        whisper_model_      = get_parameter("whisper_model").as_string();
        piper_binary_       = get_parameter("piper_binary").as_string();
        piper_model_        = get_parameter("piper_model").as_string();
        sample_rate_        = get_parameter("sample_rate").as_int();
        record_seconds_     = get_parameter("record_seconds").as_int();
        energy_threshold_   = static_cast<float>(get_parameter("energy_threshold").as_double());
        whisper_threads_    = get_parameter("whisper_threads").as_int();
        whisper_language_   = get_parameter("whisper_language").as_string();
        response_timeout_s_ = get_parameter("response_timeout_s").as_int();
        asr_cooldown_s_     = get_parameter("asr_cooldown_s").as_int();
        vad_snr_ratio_      = static_cast<float>(get_parameter("vad_snr_ratio").as_double());
        tts_sink_           = get_parameter("tts_sink").as_string();
    }

    // ── Initialisation ────────────────────────────────────────────────────────

    void init_whisper() {
        whisper_context_params cparams = whisper_context_default_params();
        cparams.use_gpu    = true;
        cparams.gpu_device = 0;

        whisper_ctx_ = whisper_init_from_file_with_params(whisper_model_.c_str(), cparams);
        if (!whisper_ctx_) {
            RCLCPP_FATAL(get_logger(), "Failed to load whisper model: %s", whisper_model_.c_str());
            throw std::runtime_error("whisper init failed");
        }
        RCLCPP_INFO(get_logger(), "Whisper model loaded (GPU enabled)");
    }

    // ── Temporal energy ratio VAD ─────────────────────────────────────────────
    //
    // Splits the captured buffer into 100ms windows and computes per-window RMS.
    // Returns true if max_window_rms / min_window_rms exceeds vad_snr_ratio_.
    //
    // Rationale: steady background noise (HVAC fans) has near-constant energy
    // across time → low ratio. Speech has loud consonants/vowels interspersed with
    // pauses → high ratio. No external library required; runs in microseconds on CPU.
    bool vad_has_speech(const std::vector<int16_t>& pcm) const {
        const int window_samples = sample_rate_ / 10;  // 100ms
        const int n_windows      = static_cast<int>(pcm.size()) / window_samples;
        if (n_windows < 4) return false;  // buffer too short to make a VAD decision — reject

        // Skip the first second of every recording.  pw-record has a ~0.5-1.0s
        // startup ramp: output file opens, audio device initialises, then noise
        // ramps from near-zero up to the real noise floor.  Those transition
        // windows (RMS 50-1500) end up as the minimum of the sorted distribution
        // and inflate max/min ratio for completely steady background noise, causing
        // the VAD to pass every capture even when the room is silent.
        // Capped at 25% of the recording so we never discard most of a short buffer.
        const int startup_skip = std::min(n_windows / 4, sample_rate_ / window_samples);

        std::vector<float> rms_vals;
        rms_vals.reserve(n_windows - startup_skip);

        for (int i = startup_skip; i < n_windows; ++i) {
            double sum_sq = 0.0;
            const int offset = i * window_samples;
            for (int j = 0; j < window_samples; ++j) {
                const float s = static_cast<float>(pcm[offset + j]);
                sum_sq += s * s;
            }
            const float rms = static_cast<float>(std::sqrt(sum_sq / window_samples));
            if (rms < 10.0f) continue;  // truly silent frame (device not yet open)
            rms_vals.push_back(rms);
        }

        const int n_valid = static_cast<int>(rms_vals.size());
        if (n_valid < 2) return false;

        std::sort(rms_vals.begin(), rms_vals.end());

        // Trim bottom 25% and top 10%.
        // Bottom 25%: mops up any late-arriving startup transition frames that
        //             survived the skip (device ramp can extend past 1s on cold starts).
        // Top 10%:    removes brief click/transient spikes.
        // After both trims, steady noise has min≈max≈floor → ratio≈1.0 (fails).
        // Real speech has loud peaks against quiet pauses → ratio >> 1.7 (passes).
        const int trim_low  = std::max(1, n_valid / 4);   // 25%
        const int trim_high = std::max(1, n_valid / 10);  // 10%
        if (n_valid - trim_low - trim_high < 2) return false;

        const float min_rms = rms_vals[trim_low];
        const float max_rms = rms_vals[n_valid - 1 - trim_high];

        RCLCPP_DEBUG(get_logger(),
            "[VAD] skip=%d valid=%d lo=%d hi=%d min=%.0f max=%.0f ratio=%.2f thr=%.1f",
            startup_skip, n_valid, trim_low, trim_high, min_rms, max_rms,
            max_rms / min_rms, vad_snr_ratio_);
        return (max_rms / min_rms) > vad_snr_ratio_;
    }

    // ── Audio ingestion ───────────────────────────────────────────────────────

    // Frames arrive from the Pi 5 capture node. Append to the accumulator unless
    // we are currently speaking (drop our own TTS-era audio). Runs on the executor
    // thread; accum_ is shared with the audio_loop processing thread.
    void audio_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        last_audio_rx_ = std::chrono::steady_clock::now();
        if (is_speaking_.load()) return;  // ignore frames during TTS playback
        std::lock_guard<std::mutex> lock(accum_mutex_);
        accum_.insert(accum_.end(), msg->data.begin(), msg->data.end());
        // Bound growth if the processing thread ever stalls — keep at most 2 windows.
        const size_t cap = static_cast<size_t>(sample_rate_) * record_seconds_ * 2;
        if (accum_.size() > cap) {
            accum_.erase(accum_.begin(), accum_.end() - cap);
        }
    }

    // Processing loop (background thread): pulls full windows from accum_ and runs
    // the energy gate / VAD / Whisper pipeline — identical logic to the old
    // pw-record path, just sourced from the streamed accumulator.
    void audio_loop() {
        const size_t window = static_cast<size_t>(sample_rate_) * record_seconds_;
        bool was_speaking = false;
        last_audio_rx_ = std::chrono::steady_clock::now();
        auto last_stale_warn = std::chrono::steady_clock::now();

        while (running_ && rclcpp::ok()) {
            if (is_speaking_.load()) {
                was_speaking = true;
                if (std::chrono::steady_clock::now() > speak_deadline_) {
                    RCLCPP_WARN(get_logger(), "[WATCHDOG] No response received — resuming listening");
                    is_speaking_ = false;
                }
                { std::lock_guard<std::mutex> lock(accum_mutex_); accum_.clear(); }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            if (was_speaking) {
                was_speaking = false;
                last_asr_run_ = std::chrono::steady_clock::time_point{};
                // Discard anything buffered during/just after TTS so the next
                // utterance starts on a clean window.
                std::lock_guard<std::mutex> lock(accum_mutex_);
                accum_.clear();
            }

            // Pull one full window from the accumulator, if ready.
            std::vector<int16_t> buf;
            {
                std::lock_guard<std::mutex> lock(accum_mutex_);
                if (accum_.size() >= window) {
                    buf.assign(accum_.begin(), accum_.begin() + window);
                    accum_.erase(accum_.begin(), accum_.begin() + window);
                }
            }

            if (buf.empty()) {
                // No full window yet. Warn (throttled) if the Pi 5 stream has gone silent.
                const auto now = std::chrono::steady_clock::now();
                if (now - last_audio_rx_.load() > std::chrono::seconds(5) &&
                    now - last_stale_warn > std::chrono::seconds(10)) {
                    RCLCPP_WARN(get_logger(),
                        "[AUDIO] no frames on /audio/mic_raw for >5s — is the Pi 5 capture node running?");
                    last_stale_warn = now;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }

            if (is_speaking_.load()) continue;

            const float energy = compute_rms_energy(buf);
            if (energy < energy_threshold_) {
                RCLCPP_INFO(get_logger(), "[VAD] low energy %.0f < %.0f — skipping",
                            energy, energy_threshold_);
                continue;
            }

            // Temporal energy ratio VAD — gates Whisper on CPU before touching the GPU.
            // Prevents the GPU spike (idle→1575MHz) that causes desktop jitter on every
            // ambient-noise capture.
            // Bypassed during registration: short single words ("Logan", "yes", "no")
            // lack the start/stop/pause structure the ratio VAD expects, so it wrongly
            // classifies them as "steady noise". The energy gate above still applies.
            if (!expecting_name_.load() && !vad_has_speech(buf)) {
                RCLCPP_INFO(get_logger(), "[VAD] steady noise (RMS %.0f) — skipping Whisper", energy);
                continue;
            }

            // Speech confirmed — convert to float32 for Whisper
            const std::vector<float> pcmf32 = pcm16_to_float32(buf);

            RCLCPP_INFO(get_logger(), "[PROCESSING] Energy %.0f — transcribing...", energy);
            last_asr_run_ = std::chrono::steady_clock::now();

            const std::string text = run_asr(pcmf32);

            if (text.empty() || text.size() < 3) {
                RCLCPP_DEBUG(get_logger(), "[ASR] Short/empty result — skipping");
                // Guard against stuck loops where steady background noise repeatedly
                // passes VAD but Whisper finds no speech.  After 3 consecutive empty
                // results, pause 4s so the noise condition can change.  The counter
                // resets whenever a real transcription is published.
                if (++consecutive_empty_asr_ >= 3) {
                    RCLCPP_INFO(get_logger(),
                        "[VAD] %d consecutive empty ASR results — pausing 4s to break noise loop",
                        consecutive_empty_asr_);
                    consecutive_empty_asr_ = 0;
                    for (int i = 0; i < 40 && running_.load() && rclcpp::ok(); ++i) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                }
                continue;
            }
            consecutive_empty_asr_ = 0;

            if (is_noise_label(text)) {
                RCLCPP_DEBUG(get_logger(), "[ASR] Noise label filtered: %s", text.c_str());
                continue;
            }

            // Require at least 2 words — single-word results are almost always hallucinations
            {
                size_t word_count = 0;
                bool in_word = false;
                for (char c : text) {
                    if (c != ' ' && !in_word) { ++word_count; in_word = true; }
                    else if (c == ' ')         { in_word = false; }
                }
                // During registration we expect single-word answers ("Logan",
                // "yes", "no"), so the 2-word minimum is bypassed in that window.
                if (word_count < 2 && !expecting_name_.load()) {
                    RCLCPP_INFO(get_logger(), "[ASR] single-word result filtered: '%s'", text.c_str());
                    continue;
                }
            }

            RCLCPP_INFO(get_logger(), "[USER] %s", text.c_str());

            std_msgs::msg::String msg;
            msg.data = text;
            text_pub_->publish(msg);

            is_speaking_ = true;
            speak_deadline_ = std::chrono::steady_clock::now() +
                              std::chrono::seconds(response_timeout_s_);
        }
    }

    // ── ASR ───────────────────────────────────────────────────────────────────

    std::string run_asr(const std::vector<float>& pcmf32) {
        // Whisper requires at least 100ms (1600 samples at 16kHz) to run its
        // log-mel spectrogram.  This should never trigger in normal operation since
        // capture_chunk always returns a full record_seconds_ buffer, but guard
        // defensively against edge cases (e.g. pw-record exits abnormally early).
        if (pcmf32.size() < static_cast<size_t>(sample_rate_ / 10)) return {};

        whisper_full_params wparams = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);
        wparams.language         = whisper_language_.c_str();
        wparams.n_threads        = whisper_threads_;
        wparams.single_segment   = false;
        wparams.print_realtime   = false;
        wparams.print_progress   = false;
        wparams.print_timestamps = false;
        wparams.print_special    = false;
        // No initial_prompt — priming with "robot/computer" vocabulary caused
        // Whisper to generate on-topic hallucinations from ambient noise.

        if (whisper_full(whisper_ctx_, wparams, pcmf32.data(),
                         static_cast<int>(pcmf32.size())) != 0) {
            RCLCPP_ERROR(get_logger(), "whisper_full() inference failed");
            return {};
        }

        // Check average token confidence — hallucinated text has characteristically
        // low token probabilities
        float total_prob  = 0.0f;
        int   token_count = 0;
        const int n_segments = whisper_full_n_segments(whisper_ctx_);
        for (int i = 0; i < n_segments; ++i) {
            const int n_tokens = whisper_full_n_tokens(whisper_ctx_, i);
            for (int j = 0; j < n_tokens; ++j) {
                const whisper_token_data td = whisper_full_get_token_data(whisper_ctx_, i, j);
                if (td.id < whisper_token_eot(whisper_ctx_)) {
                    total_prob += td.p;
                    ++token_count;
                }
            }
        }
        if (token_count > 0 && (total_prob / token_count) < 0.30f) {
            RCLCPP_DEBUG(get_logger(), "[ASR] Low avg token confidence %.2f — likely hallucination",
                total_prob / token_count);
            return {};
        }

        std::string result;
        for (int i = 0; i < n_segments; ++i) {
            const float no_speech_prob = whisper_full_get_segment_no_speech_prob(whisper_ctx_, i);
            if (no_speech_prob > 0.5f) {
                RCLCPP_DEBUG(get_logger(), "[ASR] Segment %d no_speech_prob=%.2f — skipping", i, no_speech_prob);
                continue;
            }
            result += whisper_full_get_segment_text(whisper_ctx_, i);
        }

        const auto first = result.find_first_not_of(" \t\n\r");
        if (first == std::string::npos) return {};
        const auto last = result.find_last_not_of(" \t\n\r");
        return result.substr(first, last - first + 1);
    }

    // ── TTS ───────────────────────────────────────────────────────────────────

    void response_callback(const std_msgs::msg::String::SharedPtr msg) {
        const std::string text = msg->data;
        if (text.empty() || text.size() < 2) {
            // Empty ack (e.g. brain is sleeping) — clear the speaking flag immediately
            // so the listening loop isn't blocked for the full response_timeout_s.
            speak_deadline_ = std::chrono::steady_clock::time_point::max();
            is_speaking_ = false;
            return;
        }

        RCLCPP_INFO(get_logger(), "[JUPITER] %s", text.c_str());

        is_speaking_ = true;
        speak(text);
        // Pause so room echo from TTS playback dies down before next capture.
        // 1200ms needed for longer registration prompts — shorter caused echo
        // bleed into the recording window, making Whisper transcribe TTS echo
        // instead of the user's name response.
        std::this_thread::sleep_for(std::chrono::milliseconds(1200));
        // Signal display to return to LISTENING — exact timing, no word-count estimate
        tts_done_pub_->publish(std_msgs::msg::Empty{});
        // Reset deadline so a stale timeout from a previous query doesn't fire
        // the watchdog the next time is_speaking_ goes true (e.g. an autonomous greeting)
        speak_deadline_ = std::chrono::steady_clock::time_point::max();
        // Reset empty ASR counter so post-TTS echo doesn't push us into the 4s pause loop.
        // Without this, room echo after TTS gives Whisper 1-3 empty results, consecutive_empty_asr_
        // hits 3, and the loop pauses 4s — making the robot appear stuck after every response.
        consecutive_empty_asr_ = 0;
        is_speaking_ = false;

        RCLCPP_INFO(get_logger(), "[LISTENING] Ready");
    }

    void speak(const std::string& text) {
        const std::string safe = sanitize_for_shell(text);
        // Route playback to a specific sink (HDMI) when tts_sink_ is set, so TTS
        // does not play through the ReSpeaker and wedge its mic capture.
        const std::string target = tts_sink_.empty() ? "" : (" --target " + tts_sink_);
        const std::string cmd =
            "echo '" + safe + "' | " +
            piper_binary_ + " --model " + piper_model_ +
            " --output_raw 2>/dev/null"
            " | pw-cat --playback" + target +
            " --rate 22050 --format s16 --channels 1 - 2>/dev/null";

        // Fork instead of std::system() so we can kill the child on SIGINT
        // without blocking the ROS spin thread for the full TTS duration.
        const pid_t pid = ::fork();
        if (pid < 0) return;
        if (pid == 0) {
            ::execl("/bin/sh", "sh", "-c", cmd.c_str(), nullptr);
            ::_exit(1);
        }
        tts_pid_.store(pid);

        bool finished = false;
        while (running_.load() && rclcpp::ok()) {
            int status;
            if (::waitpid(pid, &status, WNOHANG) == pid) { finished = true; break; }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (!finished) {
            ::kill(pid, SIGKILL);  // SIGKILL on shutdown — SA_RESTART blocks SIGTERM+waitpid
            ::waitpid(pid, nullptr, 0);
        }
        tts_pid_.store(-1);
    }

    // ── Members ───────────────────────────────────────────────────────────────

    // Whisper
    whisper_context* whisper_ctx_{nullptr};
    std::string      whisper_model_;
    std::string      whisper_language_{"en"};
    int              whisper_threads_{4};

    int                    sample_rate_{16000};
    int                    record_seconds_{5};
    float                  energy_threshold_{300.0f};
    std::atomic<pid_t>     tts_pid_{-1};  // TTS child PID — destructor kills it

    // Audio accumulator — fed by audio_callback (executor thread), drained in
    // window-sized chunks by audio_loop (processing thread).
    std::vector<int16_t>   accum_;
    std::mutex             accum_mutex_;
    std::atomic<std::chrono::steady_clock::time_point> last_audio_rx_{std::chrono::steady_clock::time_point{}};

    // Piper TTS
    std::string piper_binary_;
    std::string piper_model_;
    std::string tts_sink_;

    // Threading / state
    std::thread            audio_thread_;
    std::atomic<bool>      running_{true};
    std::atomic<bool>      is_speaking_{false};
    int                    response_timeout_s_{30};
    int                    asr_cooldown_s_{0};
    std::chrono::steady_clock::time_point last_asr_run_{std::chrono::steady_clock::time_point{}};
    std::chrono::steady_clock::time_point speak_deadline_{std::chrono::steady_clock::time_point::max()};

    // Temporal energy ratio VAD
    float vad_snr_ratio_{1.7f};
    int   consecutive_empty_asr_{0};  // reset after good transcription; triggers cooldown at 3

    // ROS2
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    text_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr     tts_done_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr response_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   expecting_name_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr audio_sub_;
    std::atomic<bool>                                      expecting_name_{false};
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JupiterVoice>());
    rclcpp::shutdown();
    return 0;
}
