// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// Jupiter Wake Word Node
// arecord capture → sherpa-onnx streaming keyword spotter → /voice/wake
// Runs entirely on CPU; no GPU DMA, no effect on LiDAR USB timing.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "sherpa-onnx/c-api/c-api.h"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

std::string default_path(const std::string& rel) {
    const char* home = std::getenv("HOME");
    return std::string(home ? home : "/home/jupiter") + "/" + rel;
}

} // namespace


class JupiterWakeWord : public rclcpp::Node {
public:
    explicit JupiterWakeWord() : Node("jupiter_wakeword") {
        declare_parameters();
        init_spotter();
        init_capture();

        wake_pub_ = create_publisher<std_msgs::msg::String>("/voice/wake", 10);

        audio_thread_ = std::thread(&JupiterWakeWord::audio_loop, this);

        RCLCPP_INFO(get_logger(),
            "Jupiter Wake Word online — listening for 'Hey Jupiter' on %s",
            capture_device_.c_str());
    }

    ~JupiterWakeWord() {
        running_ = false;
        // Kill arecord first — causes pipe EOF, unblocking the audio thread
        if (arecord_pid_ > 0) {
            ::kill(arecord_pid_, SIGTERM);
            ::waitpid(arecord_pid_, nullptr, 0);
            arecord_pid_ = -1;
        }
        if (pipe_fd_ >= 0) {
            close(pipe_fd_);
            pipe_fd_ = -1;
        }
        if (audio_thread_.joinable()) audio_thread_.join();
        if (stream_)  SherpaOnnxDestroyOnlineStream(stream_);
        if (spotter_) SherpaOnnxDestroyKeywordSpotter(spotter_);
    }

private:
    // ── Parameters ────────────────────────────────────────────────────────────

    void declare_parameters() {
        const std::string model_dir =
            default_path("jupitercpp_ws/sherpa-onnx-kws-zipformer-gigaspeech-3.3M-2024-01-01");

        declare_parameter("model_dir",      model_dir);
        declare_parameter("keywords_file",
            default_path("jupitercpp_ws/models/wakeword/keywords.txt"));
        declare_parameter("capture_device", std::string("default"));
        declare_parameter("threshold",      0.1);
        declare_parameter("score",          3.0);
        declare_parameter("num_threads",    1);
        declare_parameter("cooldown_s",     3.0);

        model_dir_      = get_parameter("model_dir").as_string();
        keywords_file_  = get_parameter("keywords_file").as_string();
        capture_device_ = get_parameter("capture_device").as_string();
        threshold_      = static_cast<float>(get_parameter("threshold").as_double());
        score_          = static_cast<float>(get_parameter("score").as_double());
        num_threads_    = get_parameter("num_threads").as_int();
        cooldown_       = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(
                                  get_parameter("cooldown_s").as_double()));
    }

    // ── arecord capture init ──────────────────────────────────────────────────

    void init_capture() {
        int pipefd[2];
        if (pipe(pipefd) < 0) {
            throw std::runtime_error(
                std::string("pipe() failed: ") + strerror(errno));
        }

        const pid_t pid = fork();
        if (pid < 0) {
            close(pipefd[0]);
            close(pipefd[1]);
            throw std::runtime_error(
                std::string("fork() failed: ") + strerror(errno));
        }

        if (pid == 0) {
            // Child: redirect stdout to write end of pipe
            close(pipefd[0]);
            dup2(pipefd[1], STDOUT_FILENO);
            close(pipefd[1]);

            const int devnull = open("/dev/null", O_WRONLY);
            if (devnull >= 0) {
                dup2(devnull, STDERR_FILENO);
                close(devnull);
            }

            execlp("arecord", "arecord",
                   "-D", capture_device_.c_str(),
                   "-r", "16000",
                   "-f", "S16_LE",
                   "-c", "1",
                   "-t", "raw",
                   "-", nullptr);
            _exit(1);
        }

        // Parent: close write end, keep read end
        close(pipefd[1]);
        pipe_fd_     = pipefd[0];
        arecord_pid_ = pid;

        RCLCPP_INFO(get_logger(), "arecord capture started (PID %d) on '%s' @ 16kHz mono",
                    pid, capture_device_.c_str());
    }

    // ── Sherpa-ONNX keyword spotter init ──────────────────────────────────────

    void init_spotter() {
        const std::string encoder = model_dir_ +
            "/encoder-epoch-12-avg-2-chunk-16-left-64.int8.onnx";
        const std::string decoder = model_dir_ +
            "/decoder-epoch-12-avg-2-chunk-16-left-64.onnx";
        const std::string joiner  = model_dir_ +
            "/joiner-epoch-12-avg-2-chunk-16-left-64.int8.onnx";
        const std::string tokens  = model_dir_ + "/tokens.txt";

        SherpaOnnxKeywordSpotterConfig config;
        std::memset(&config, 0, sizeof(config));

        config.feat_config.sample_rate = 16000;
        config.feat_config.feature_dim = 80;

        config.model_config.transducer.encoder = encoder.c_str();
        config.model_config.transducer.decoder = decoder.c_str();
        config.model_config.transducer.joiner  = joiner.c_str();
        config.model_config.tokens             = tokens.c_str();
        config.model_config.provider           = "cpu";
        config.model_config.num_threads        = num_threads_;

        config.max_active_paths    = 4;
        config.num_trailing_blanks = 1;
        config.keywords_score      = score_;
        config.keywords_threshold  = threshold_;
        config.keywords_file       = keywords_file_.c_str();

        spotter_ = SherpaOnnxCreateKeywordSpotter(&config);
        if (!spotter_) {
            throw std::runtime_error("Failed to create sherpa-onnx keyword spotter");
        }

        stream_ = SherpaOnnxCreateKeywordStream(spotter_);
        if (!stream_) {
            SherpaOnnxDestroyKeywordSpotter(spotter_);
            spotter_ = nullptr;
            throw std::runtime_error("Failed to create keyword stream");
        }

        RCLCPP_INFO(get_logger(), "Keyword spotter loaded — model: %s  threshold: %.2f",
                    model_dir_.c_str(), threshold_);
        RCLCPP_INFO(get_logger(), "Keywords file: %s", keywords_file_.c_str());
    }

    // ── Audio loop (background thread) ────────────────────────────────────────

    // Read exactly `bytes` bytes from pipe into buf, returning false on EOF/error.
    bool pipe_read_exact(char* buf, size_t bytes) {
        size_t done = 0;
        while (done < bytes) {
            if (!running_) return false;
            const ssize_t n = read(pipe_fd_, buf + done, bytes - done);
            if (n < 0) {
                if (errno == EINTR) continue;
                return false;
            }
            if (n == 0) return false;  // EOF — arecord exited
            done += static_cast<size_t>(n);
        }
        return true;
    }

    void audio_loop() {
        constexpr int kChunk = 1600;  // 100 ms at 16 kHz
        std::vector<int16_t> raw(kChunk);
        std::vector<float>   pcmf(kChunk);
        int diag_counter = 0;

        RCLCPP_INFO(get_logger(), "[AUDIO] Thread started — reading from arecord pipe");

        while (running_) {
            if (!pipe_read_exact(reinterpret_cast<char*>(raw.data()),
                                 kChunk * sizeof(int16_t))) {
                if (running_) {
                    RCLCPP_ERROR(get_logger(), "[AUDIO] Pipe read failed — arecord may have exited");
                }
                break;
            }

            // Convert int16 → float32 [-1, 1]
            float rms = 0.0f;
            for (int i = 0; i < kChunk; ++i) {
                pcmf[i] = raw[i] / 32768.0f;
                rms += pcmf[i] * pcmf[i];
            }
            rms = std::sqrt(rms / kChunk);

            // Log RMS every 50 chunks (~5 s) to confirm audio is flowing
            if (++diag_counter >= 50) {
                diag_counter = 0;
                RCLCPP_INFO(get_logger(), "[AUDIO] RMS %.4f  device: %s",
                            rms, capture_device_.c_str());
            }

            SherpaOnnxOnlineStreamAcceptWaveform(stream_, 16000, pcmf.data(), kChunk);

            while (SherpaOnnxIsKeywordStreamReady(spotter_, stream_)) {
                SherpaOnnxDecodeKeywordStream(spotter_, stream_);
            }

            const SherpaOnnxKeywordResult* result =
                SherpaOnnxGetKeywordResult(spotter_, stream_);

            if (result && result->keyword && result->keyword[0] != '\0') {
                const auto now = std::chrono::steady_clock::now();
                if (now - last_detection_ >= cooldown_) {
                    last_detection_ = now;
                    RCLCPP_INFO(get_logger(), "[WAKE] 'Hey Jupiter' detected — arming voice");
                    std_msgs::msg::String msg;
                    msg.data = "hey_jupiter";
                    wake_pub_->publish(msg);
                }
                SherpaOnnxResetKeywordStream(spotter_, stream_);
            }

            if (result) SherpaOnnxDestroyKeywordResult(result);
        }
    }

    // ── Members ───────────────────────────────────────────────────────────────

    // Parameters
    std::string model_dir_;
    std::string keywords_file_;
    std::string capture_device_;
    float       threshold_{0.1f};
    float       score_{3.0f};
    int         num_threads_{1};
    std::chrono::steady_clock::duration cooldown_{std::chrono::seconds(3)};

    // Capture pipe
    int   pipe_fd_{-1};
    pid_t arecord_pid_{-1};

    // Sherpa-ONNX
    const SherpaOnnxKeywordSpotter* spotter_{nullptr};
    const SherpaOnnxOnlineStream*   stream_{nullptr};

    // Threading
    std::thread       audio_thread_;
    std::atomic<bool> running_{true};
    std::chrono::steady_clock::time_point last_detection_{};

    // ROS2
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr wake_pub_;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JupiterWakeWord>());
    rclcpp::shutdown();
    return 0;
}
