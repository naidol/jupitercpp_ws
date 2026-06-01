// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// mic_capture_node — runs on the Raspberry Pi 5 sub-node.
//
// Opens the ReSpeaker XVF3800 via ALSA ONCE and reads continuously on a
// dedicated thread, publishing fixed-size 16 kHz mono S16 frames on
// /audio/mic_raw. The continuous single-stream design avoids the per-chunk
// open/close cycling that wedged the device on the Jetson Thor's USB controller.
//
// Keeps the Pi 5 footprint minimal (objective D): ALSA + rclcpp + std_msgs only,
// no PipeWire, no resampling libraries, strict capture/exec thread separation.
//
// The Jetson Thor subscribes to /audio/mic_raw and feeds the existing
// energy-gate / VAD / Whisper ASR pipeline exactly as if the mic were local.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

#include <alsa/asoundlib.h>

#include <atomic>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

class MicCaptureNode : public rclcpp::Node {
public:
    MicCaptureNode() : Node("mic_capture_node") {
        // plughw: lets ALSA downmix the ReSpeaker's native 16 kHz stereo to the
        // mono the ASR pipeline expects. Device addressed by card NAME ("Array")
        // so it survives card-index reordering across reboots.
        device_       = declare_parameter("alsa_device", std::string("plughw:Array,0"));
        sample_rate_  = declare_parameter("sample_rate", 16000);
        const int frame_ms = declare_parameter("frame_ms", 100);
        frame_samples_ = sample_rate_ * frame_ms / 1000;

        // Best-Effort, shallow history — real-time voice: prefer low latency over
        // guaranteed delivery on the dedicated point-to-point link.
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.best_effort();
        pub_ = create_publisher<std_msgs::msg::Int16MultiArray>("/audio/mic_raw", qos);

        if (!open_alsa()) {
            throw std::runtime_error("ALSA capture open failed for " + device_);
        }

        running_ = true;
        capture_thread_ = std::thread(&MicCaptureNode::capture_loop, this);

        RCLCPP_INFO(get_logger(),
            "Mic capture online — %s @ %d Hz mono, %d-sample (%d ms) frames on /audio/mic_raw",
            device_.c_str(), sample_rate_, frame_samples_, frame_ms);
    }

    ~MicCaptureNode() override {
        running_ = false;
        if (capture_thread_.joinable()) capture_thread_.join();
        if (pcm_) snd_pcm_close(pcm_);
    }

private:
    bool open_alsa() {
        int err = snd_pcm_open(&pcm_, device_.c_str(), SND_PCM_STREAM_CAPTURE, 0);
        if (err < 0) {
            RCLCPP_ERROR(get_logger(), "snd_pcm_open(%s): %s",
                         device_.c_str(), snd_strerror(err));
            return false;
        }

        snd_pcm_hw_params_t* hw;
        snd_pcm_hw_params_alloca(&hw);
        snd_pcm_hw_params_any(pcm_, hw);
        snd_pcm_hw_params_set_access(pcm_, hw, SND_PCM_ACCESS_RW_INTERLEAVED);
        snd_pcm_hw_params_set_format(pcm_, hw, SND_PCM_FORMAT_S16_LE);
        snd_pcm_hw_params_set_channels(pcm_, hw, 1);
        unsigned int rate = static_cast<unsigned int>(sample_rate_);
        snd_pcm_hw_params_set_rate_near(pcm_, hw, &rate, nullptr);

        err = snd_pcm_hw_params(pcm_, hw);
        if (err < 0) {
            RCLCPP_ERROR(get_logger(), "snd_pcm_hw_params: %s", snd_strerror(err));
            return false;
        }
        snd_pcm_prepare(pcm_);
        return true;
    }

    void capture_loop() {
        std::vector<int16_t> buf(frame_samples_);
        while (running_.load() && rclcpp::ok()) {
            const snd_pcm_sframes_t n = snd_pcm_readi(pcm_, buf.data(), frame_samples_);
            if (n < 0) {
                // xrun/underrun — recover and continue (1 = silent recover)
                snd_pcm_recover(pcm_, static_cast<int>(n), 1);
                continue;
            }
            if (n == 0) continue;

            std_msgs::msg::Int16MultiArray msg;
            msg.data.assign(buf.begin(), buf.begin() + n);
            pub_->publish(msg);
        }
    }

    snd_pcm_t*  pcm_{nullptr};
    std::string device_;
    int         sample_rate_{16000};
    int         frame_samples_{1600};
    std::atomic<bool> running_{false};
    std::thread capture_thread_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<MicCaptureNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("mic_capture_node"), "%s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
