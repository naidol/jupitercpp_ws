// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// Jupiter Voice Node
// PipeWire capture (pw-record via fork/exec) → whisper.cpp ASR → /voice/raw_text
// /voice/response_text → piper TTS (pw-cat)

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "whisper.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <fcntl.h>
#include <fstream>
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
    // Bracketed:  [Music]  [Barking]  [Laughter]
    if (first == '[' && last == ']') return true;
    // Parenthesised: (applause)  (music)
    if (first == '(' && last == ')') return true;
    // Asterisk-wrapped: *beep*  *music*
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

        text_pub_ = create_publisher<std_msgs::msg::String>("/voice/raw_text", 10);

        response_sub_ = create_subscription<std_msgs::msg::String>(
            "/voice/response_text", 10,
            std::bind(&JupiterVoice::response_callback, this, std::placeholders::_1));

        audio_thread_ = std::thread(&JupiterVoice::audio_loop, this);

        RCLCPP_INFO(get_logger(), "Jupiter Voice online — pw-record capture + whisper.cpp ASR + piper TTS");
        RCLCPP_INFO(get_logger(), "Recording %ds chunks @ %dHz  |  model: %s",
            record_seconds_, sample_rate_, whisper_model_.c_str());
    }

    ~JupiterVoice() {
        running_ = false;
        // Kill any in-progress pw-record so the audio thread unblocks immediately
        const pid_t pid = capture_pid_.load();
        if (pid > 0) ::kill(pid, SIGTERM);
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
        declare_parameter("response_timeout_s", 30);

        whisper_model_      = get_parameter("whisper_model").as_string();
        piper_binary_       = get_parameter("piper_binary").as_string();
        piper_model_        = get_parameter("piper_model").as_string();
        sample_rate_        = get_parameter("sample_rate").as_int();
        record_seconds_     = get_parameter("record_seconds").as_int();
        energy_threshold_   = static_cast<float>(get_parameter("energy_threshold").as_double());
        whisper_threads_    = get_parameter("whisper_threads").as_int();
        whisper_language_   = get_parameter("whisper_language").as_string();
        response_timeout_s_ = get_parameter("response_timeout_s").as_int();
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

    // ── Audio capture loop (background thread) ────────────────────────────────

    void audio_loop() {
        const std::string tmp_file =
            "/tmp/jupiter_capture_" + std::to_string(getpid()) + ".raw";

        while (running_) {
            if (is_speaking_.load()) {
                if (std::chrono::steady_clock::now() > speak_deadline_) {
                    RCLCPP_WARN(get_logger(), "[WATCHDOG] No response received — resuming listening");
                    is_speaking_ = false;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            const std::vector<int16_t> buf = capture_chunk(tmp_file);
            if (buf.empty()) continue;

            if (is_speaking_.load()) continue;

            const float energy = compute_rms_energy(buf);
            RCLCPP_DEBUG(get_logger(), "[VAD] Energy %.0f  threshold %.0f", energy, energy_threshold_);
            if (energy < energy_threshold_) continue;

            RCLCPP_INFO(get_logger(), "[PROCESSING] Energy %.0f — transcribing...", energy);

            const std::string text = run_asr(buf);

            if (text.empty() || text.size() < 3) {
                RCLCPP_DEBUG(get_logger(), "[ASR] Short/empty result — skipping");
                continue;
            }

            // Drop Whisper noise labels — non-speech sounds transcribed as [Label] or (label)
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
                if (word_count < 2) {
                    RCLCPP_DEBUG(get_logger(), "[ASR] Single-word result filtered: %s", text.c_str());
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
        std::remove(tmp_file.c_str());
    }

    // Fork pw-record directly — no shell wrapper, so we hold the PID and can
    // kill it immediately on shutdown without waiting for the full chunk duration.
    std::vector<int16_t> capture_chunk(const std::string& tmp_file) {
        const pid_t pid = fork();
        if (pid == 0) {
            // Child: redirect stderr to /dev/null, exec pw-record
            const int devnull = open("/dev/null", O_WRONLY);
            if (devnull >= 0) {
                dup2(devnull, STDERR_FILENO);
                close(devnull);
            }
            execlp("pw-record", "pw-record",
                   "--rate",     std::to_string(sample_rate_).c_str(),
                   "--format",   "s16",
                   "--channels", "1",
                   tmp_file.c_str(),
                   nullptr);
            _exit(1);
        }
        if (pid < 0) return {};

        capture_pid_.store(pid);

        // Poll until record_seconds_ elapsed OR shutdown requested
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::seconds(record_seconds_);
        while (running_.load()) {
            int status;
            if (waitpid(pid, &status, WNOHANG) != 0) {
                capture_pid_.store(-1);
                break;
            }
            if (std::chrono::steady_clock::now() >= deadline) {
                ::kill(pid, SIGTERM);
                waitpid(pid, nullptr, 0);
                capture_pid_.store(-1);
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        if (!running_.load()) {
            ::kill(pid, SIGTERM);
            waitpid(pid, nullptr, 0);
            capture_pid_.store(-1);
            std::remove(tmp_file.c_str());
            return {};
        }

        std::ifstream f(tmp_file, std::ios::binary | std::ios::ate);
        if (!f) return {};
        const auto bytes = static_cast<std::size_t>(f.tellg());
        if (bytes < 2) return {};
        f.seekg(0);
        std::vector<int16_t> buf(bytes / 2);
        f.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(bytes));
        std::remove(tmp_file.c_str());
        return buf;
    }

    // ── ASR ───────────────────────────────────────────────────────────────────

    std::string run_asr(const std::vector<int16_t>& pcm) {
        const std::vector<float> pcmf32 = pcm16_to_float32(pcm);

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

        // Check average token confidence across all segments — hallucinated text
        // from ambient noise has characteristically low token probabilities
        float total_prob  = 0.0f;
        int   token_count = 0;
        const int n_segments = whisper_full_n_segments(whisper_ctx_);
        for (int i = 0; i < n_segments; ++i) {
            const int n_tokens = whisper_full_n_tokens(whisper_ctx_, i);
            for (int j = 0; j < n_tokens; ++j) {
                const whisper_token_data td = whisper_full_get_token_data(whisper_ctx_, i, j);
                if (td.id < whisper_token_eot(whisper_ctx_)) {  // skip special tokens
                    total_prob += td.p;
                    ++token_count;
                }
            }
        }
        if (token_count > 0 && (total_prob / token_count) < 0.40f) {
            RCLCPP_DEBUG(get_logger(), "[ASR] Low avg token confidence %.2f — likely hallucination",
                total_prob / token_count);
            return {};
        }

        std::string result;
        for (int i = 0; i < n_segments; ++i) {
            // Skip segments Whisper itself flags as likely non-speech
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
        if (text.empty() || text.size() < 2) return;

        RCLCPP_INFO(get_logger(), "[JUPITER] %s", text.c_str());

        is_speaking_ = true;
        speak(text);
        // Brief pause so room echo from TTS playback dies down before next capture
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        is_speaking_ = false;

        RCLCPP_INFO(get_logger(), "[LISTENING] Ready");
    }

    void speak(const std::string& text) {
        const std::string safe = sanitize_for_shell(text);
        const std::string cmd =
            "echo '" + safe + "' | " +
            piper_binary_ + " --model " + piper_model_ +
            " --output_raw 2>/dev/null"
            " | pw-cat --playback --rate 22050 --format s16 --channels 1 - 2>/dev/null";
        [[maybe_unused]] int rc = std::system(cmd.c_str());
    }

    // ── Members ───────────────────────────────────────────────────────────────

    // Whisper
    whisper_context* whisper_ctx_{nullptr};
    std::string      whisper_model_;
    std::string      whisper_language_{"en"};
    int              whisper_threads_{4};

    // Capture
    int                    sample_rate_{16000};
    int                    record_seconds_{5};
    float                  energy_threshold_{300.0f};
    std::atomic<pid_t>     capture_pid_{-1};

    // Piper TTS
    std::string piper_binary_;
    std::string piper_model_;

    // Threading / state
    std::thread            audio_thread_;
    std::atomic<bool>      running_{true};
    std::atomic<bool>      is_speaking_{false};
    int                    response_timeout_s_{30};
    std::chrono::steady_clock::time_point speak_deadline_{std::chrono::steady_clock::time_point::max()};

    // ROS2
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    text_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr response_sub_;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JupiterVoice>());
    rclcpp::shutdown();
    return 0;
}
