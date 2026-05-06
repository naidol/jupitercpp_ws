// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// Jupiter Voice Node
// Audio capture (ALSA/ReSpeaker) → whisper.cpp ASR → /voice/raw_text
// /voice/response_text → piper TTS

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <alsa/asoundlib.h>
#include "whisper.h"

#include <atomic>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <thread>
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
        init_alsa();

        text_pub_ = create_publisher<std_msgs::msg::String>("/voice/raw_text", 10);

        response_sub_ = create_subscription<std_msgs::msg::String>(
            "/voice/response_text", 10,
            std::bind(&JupiterVoice::response_callback, this, std::placeholders::_1));

        audio_thread_ = std::thread(&JupiterVoice::audio_loop, this);

        RCLCPP_INFO(get_logger(), "Jupiter Voice online — whisper.cpp ASR + piper TTS");
        RCLCPP_INFO(get_logger(), "Listening on %s  |  model: %s",
            alsa_device_.c_str(), whisper_model_.c_str());
    }

    ~JupiterVoice() {
        running_ = false;
        if (audio_thread_.joinable()) {
            audio_thread_.join();
        }
        if (pcm_handle_) {
            snd_pcm_drain(pcm_handle_);
            snd_pcm_close(pcm_handle_);
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
        declare_parameter("alsa_device",          std::string("plughw:2,0"));
        declare_parameter("playback_device",      std::string("plughw:2,0"));
        declare_parameter("sample_rate",          16000);
        declare_parameter("record_seconds",       5);
        declare_parameter("energy_threshold",     400.0);
        declare_parameter("whisper_threads",      4);
        declare_parameter("whisper_language",     std::string("en"));

        whisper_model_    = get_parameter("whisper_model").as_string();
        piper_binary_     = get_parameter("piper_binary").as_string();
        piper_model_      = get_parameter("piper_model").as_string();
        alsa_device_      = get_parameter("alsa_device").as_string();
        playback_device_  = get_parameter("playback_device").as_string();
        sample_rate_      = get_parameter("sample_rate").as_int();
        record_seconds_   = get_parameter("record_seconds").as_int();
        energy_threshold_ = static_cast<float>(get_parameter("energy_threshold").as_double());
        whisper_threads_  = get_parameter("whisper_threads").as_int();
        whisper_language_ = get_parameter("whisper_language").as_string();
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

    void init_alsa() {
        int err = snd_pcm_open(&pcm_handle_, alsa_device_.c_str(), SND_PCM_STREAM_CAPTURE, 0);
        if (err < 0) {
            RCLCPP_FATAL(get_logger(), "ALSA open failed [%s]: %s",
                alsa_device_.c_str(), snd_strerror(err));
            throw std::runtime_error("ALSA open failed");
        }

        // 16kHz mono 16-bit, 100ms hardware buffer
        err = snd_pcm_set_params(
            pcm_handle_,
            SND_PCM_FORMAT_S16_LE,
            SND_PCM_ACCESS_RW_INTERLEAVED,
            1,                                      // mono
            static_cast<unsigned int>(sample_rate_),
            1,                                      // allow ALSA software resampling
            100000);                                // 100ms latency (µs)

        if (err < 0) {
            snd_pcm_close(pcm_handle_);
            pcm_handle_ = nullptr;
            RCLCPP_FATAL(get_logger(), "ALSA set_params failed: %s", snd_strerror(err));
            throw std::runtime_error("ALSA set_params failed");
        }
        RCLCPP_INFO(get_logger(), "ALSA capture ready: %s @ %d Hz mono",
            alsa_device_.c_str(), sample_rate_);
    }

    // ── Audio capture loop (background thread) ────────────────────────────────

    void audio_loop() {
        const int total_samples = sample_rate_ * record_seconds_;
        std::vector<int16_t> buf(total_samples);

        while (running_) {
            // Pause while TTS is playing so we don't capture Jupiter's own voice
            if (is_speaking_.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            // If reinit after TTS failed, keep retrying until the USB device releases
            if (!pcm_handle_) {
                try {
                    init_alsa();
                } catch (const std::exception&) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
                continue;
            }

            if (!capture_chunk(buf)) {
                continue;
            }

            const float energy = compute_rms_energy(buf);
            if (energy < energy_threshold_) {
                continue; // silence — no print to keep terminal clean
            }

            RCLCPP_INFO(get_logger(), "[PROCESSING] Energy %.0f — transcribing...", energy);

            const std::string text = run_asr(buf);

            if (text.empty() || text.size() < 3) {
                RCLCPP_DEBUG(get_logger(), "[ASR] Short/empty result — skipping");
                continue;
            }

            RCLCPP_INFO(get_logger(), "[USER] %s", text.c_str());

            std_msgs::msg::String msg;
            msg.data = text;
            text_pub_->publish(msg);

            // Pause capture immediately — brain is processing; TTS will clear this flag
            is_speaking_ = true;
        }
    }

    bool capture_chunk(std::vector<int16_t>& buf) {
        if (!pcm_handle_) return false;
        const auto frames = static_cast<snd_pcm_sframes_t>(buf.size());
        snd_pcm_sframes_t n = snd_pcm_readi(pcm_handle_, buf.data(), frames);

        if (n == -EPIPE) {
            // Buffer overrun — recover silently
            snd_pcm_prepare(pcm_handle_);
            return false;
        }
        if (n < 0) {
            const int rc = snd_pcm_recover(pcm_handle_, static_cast<int>(n), 0);
            if (rc < 0) {
                RCLCPP_ERROR(get_logger(), "ALSA unrecoverable error: %s", snd_strerror(rc));
            }
            return false;
        }
        return true;
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

        if (whisper_full(whisper_ctx_, wparams, pcmf32.data(),
                         static_cast<int>(pcmf32.size())) != 0) {
            RCLCPP_ERROR(get_logger(), "whisper_full() inference failed");
            return {};
        }

        std::string result;
        const int n_segments = whisper_full_n_segments(whisper_ctx_);
        for (int i = 0; i < n_segments; ++i) {
            result += whisper_full_get_segment_text(whisper_ctx_, i);
        }

        // Trim whitespace
        const auto first = result.find_first_not_of(" \t\n\r");
        if (first == std::string::npos) return {};
        const auto last = result.find_last_not_of(" \t\n\r");
        return result.substr(first, last - first + 1);
    }

    // ── TTS ───────────────────────────────────────────────────────────────────

    void response_callback(const std_msgs::msg::String::SharedPtr msg) {
        const std::string text = msg->data;
        if (text.empty() || text.size() < 2) {
            return;
        }

        RCLCPP_INFO(get_logger(), "[JUPITER] %s", text.c_str());

        is_speaking_ = true;
        speak(text);
        is_speaking_ = false;

        RCLCPP_INFO(get_logger(), "[LISTENING] Ready");
    }

    void speak(const std::string& text) {
        // Release the capture handle so aplay can open the same USB device for playback
        if (pcm_handle_) {
            snd_pcm_drop(pcm_handle_);
            snd_pcm_close(pcm_handle_);
            pcm_handle_ = nullptr;
        }

        const std::string safe = sanitize_for_shell(text);
        const std::string cmd =
            "echo '" + safe + "' | " +
            piper_binary_ + " --model " + piper_model_ +
            " --output_raw 2>/dev/null"
            " | aplay -D " + playback_device_ +
            " -r 22050 -f S16_LE -c 1 -t raw 2>/dev/null";
        [[maybe_unused]] int rc = std::system(cmd.c_str());

        // Reopen capture — retry with backoff; audio loop will also keep retrying if needed
        for (int attempt = 0; attempt < 8; ++attempt) {
            try {
                init_alsa();
                break;
            } catch (const std::exception&) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }
    }

    // ── Members ───────────────────────────────────────────────────────────────

    // Whisper
    whisper_context* whisper_ctx_{nullptr};
    std::string      whisper_model_;
    std::string      whisper_language_{"en"};
    int              whisper_threads_{4};

    // ALSA
    snd_pcm_t*  pcm_handle_{nullptr};
    std::string alsa_device_;
    int         sample_rate_{16000};
    int         record_seconds_{5};
    float       energy_threshold_{400.0f};

    // Piper TTS
    std::string piper_binary_;
    std::string piper_model_;
    std::string playback_device_{"plughw:2,0"};

    // Threading / state
    std::thread       audio_thread_;
    std::atomic<bool> running_{true};
    std::atomic<bool> is_speaking_{false};

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
