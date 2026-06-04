// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// Jupiter Face Recognition Node
// /camera/color/image_raw → YuNet detect (OpenCV CPU) + SFace embed (TensorRT CUDA) → /current_user
// /vision/trigger "capture" → saves /tmp/jupiter_view.jpg
// /vision/register "<name>" → registers new user face from current frame

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/objdetect/face.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>

#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime.h>

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>
#include <cmath>

using json = nlohmann::json;
namespace fs = std::filesystem;

// ── TensorRT logger ──────────────────────────────────────────────────────────

class TRTLogger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            fprintf(stderr, "[TRT] %s\n", msg);
        }
    }
};

// ── SFace TensorRT engine ─────────────────────────────────────────────────────
//
// SFace model: input  "data"  [1 x 3 x 112 x 112]  float32 NCHW  BGR→RGB mean/std 127.5
//              output "fc1"   [1 x 128]              float32
//
// We build the engine once from ONNX and cache the serialised plan beside the model.

class SFaceEngine {
public:
    explicit SFaceEngine(const std::string& onnx_path) {
        const std::string plan_path = onnx_path + ".trt";

        if (fs::exists(plan_path)) {
            load_engine(plan_path);
        } else {
            fprintf(stderr, "[SFace] Building TRT engine from ONNX (one-time, ~2 min)...\n");
            build_engine(onnx_path, plan_path);
        }

        setup_io_buffers();
    }

    ~SFaceEngine() {
        if (device_input_)  cudaFree(device_input_);
        if (device_output_) cudaFree(device_output_);
        if (stream_) cudaStreamDestroy(stream_);
    }

    // Returns L2-normalised 128-d embedding, or empty on failure
    std::vector<float> embed(const cv::Mat& aligned_bgr_112) {
        if (!context_ || aligned_bgr_112.empty()) return {};

        // Preprocess to match OpenCV SFace (face_recognition_sface_2021dec):
        // raw BGR, 0-255 float, NCHW — NO normalisation, NO channel swap.
        // OpenCV's FaceRecognizerSF feeds the model via cv::dnn::blobFromImage with
        // default params (scale=1.0, mean=0, swapRB=false). The previous RGB + [-1,1]
        // preprocessing distorted the embedding space so different faces scored 0.9+
        // against each other — collapsing discrimination (e.g. a different person
        // matched as the registered user). Existing profiles MUST be re-registered
        // after this change (old embeddings live in the old, distorted space).
        constexpr int kChannels = 3, kH = 112, kW = 112;
        const int pixels = kChannels * kH * kW;
        std::vector<float> host_input(pixels);

        cv::Mat fmat;
        aligned_bgr_112.convertTo(fmat, CV_32F);   // 0-255 float, BGR preserved

        // Split into planes and pack as NCHW (B, G, R order)
        std::vector<cv::Mat> planes(3);
        cv::split(fmat, planes);
        const int plane_size = kH * kW;
        for (int c = 0; c < 3; ++c) {
            std::memcpy(host_input.data() + c * plane_size,
                        planes[c].ptr<float>(), plane_size * sizeof(float));
        }

        // Copy to device
        cudaMemcpyAsync(device_input_, host_input.data(),
                        pixels * sizeof(float), cudaMemcpyHostToDevice, stream_);

        // Run inference
        void* bindings[] = {device_input_, device_output_};
        context_->executeV2(bindings);
        cudaStreamSynchronize(stream_);

        // Copy embedding back
        std::vector<float> embedding(128);
        cudaMemcpy(embedding.data(), device_output_, 128 * sizeof(float), cudaMemcpyDeviceToHost);

        // L2-normalise
        float norm = 0.0f;
        for (float v : embedding) norm += v * v;
        norm = std::sqrt(norm);
        if (norm > 1e-6f) {
            for (float& v : embedding) v /= norm;
        }

        return embedding;
    }

    bool valid() const { return static_cast<bool>(context_); }

private:
    void build_engine(const std::string& onnx_path, const std::string& plan_path) {
        auto builder = std::unique_ptr<nvinfer1::IBuilder>(
            nvinfer1::createInferBuilder(logger_));

        const auto flags = 1U << static_cast<uint32_t>(
            nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
        auto network = std::unique_ptr<nvinfer1::INetworkDefinition>(
            builder->createNetworkV2(flags));

        auto parser = std::unique_ptr<nvonnxparser::IParser>(
            nvonnxparser::createParser(*network, logger_));

        if (!parser->parseFromFile(onnx_path.c_str(),
                static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
            fprintf(stderr, "[SFace] ONNX parse failed\n");
            return;
        }

        auto config = std::unique_ptr<nvinfer1::IBuilderConfig>(
            builder->createBuilderConfig());
        config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 1ULL << 28); // 256 MB

        auto serialised = std::unique_ptr<nvinfer1::IHostMemory>(
            builder->buildSerializedNetwork(*network, *config));
        if (!serialised) {
            fprintf(stderr, "[SFace] TRT engine build failed\n");
            return;
        }

        // Cache to disk
        std::ofstream f(plan_path, std::ios::binary);
        f.write(static_cast<const char*>(serialised->data()), serialised->size());
        fprintf(stderr, "[SFace] Engine saved: %s\n", plan_path.c_str());

        // Load from serialised bytes
        create_engine_from_memory(serialised->data(), serialised->size());
    }

    void load_engine(const std::string& plan_path) {
        std::ifstream f(plan_path, std::ios::binary | std::ios::ate);
        const std::streamsize size = f.tellg();
        f.seekg(0, std::ios::beg);
        std::vector<char> buf(size);
        f.read(buf.data(), size);
        create_engine_from_memory(buf.data(), buf.size());
        fprintf(stderr, "[SFace] Engine loaded from cache: %s\n", plan_path.c_str());
    }

    void create_engine_from_memory(const void* data, size_t size) {
        runtime_ = std::unique_ptr<nvinfer1::IRuntime>(
            nvinfer1::createInferRuntime(logger_));
        engine_ = std::shared_ptr<nvinfer1::ICudaEngine>(
            runtime_->deserializeCudaEngine(data, size));
        if (engine_) {
            context_ = std::unique_ptr<nvinfer1::IExecutionContext>(
                engine_->createExecutionContext());
        }
    }

    void setup_io_buffers() {
        if (!engine_) return;
        cudaStreamCreate(&stream_);
        // Input: 1 x 3 x 112 x 112 floats
        cudaMalloc(&device_input_,  1 * 3 * 112 * 112 * sizeof(float));
        // Output: 1 x 128 floats
        cudaMalloc(&device_output_, 128 * sizeof(float));
    }

    TRTLogger logger_;
    std::unique_ptr<nvinfer1::IRuntime>          runtime_;
    std::shared_ptr<nvinfer1::ICudaEngine>       engine_;
    std::unique_ptr<nvinfer1::IExecutionContext> context_;

    void*        device_input_{nullptr};
    void*        device_output_{nullptr};
    cudaStream_t stream_{nullptr};
};

// ── Profile stored per known user ────────────────────────────────────────────

struct UserProfile {
    std::string        name;
    std::vector<float> embedding;   // 128-d L2-normalised SFace feature vector
};

// ── Node ─────────────────────────────────────────────────────────────────────

class JupiterFaceRecognition : public rclcpp::Node {
public:
    explicit JupiterFaceRecognition() : Node("jupiter_face_recognition") {
        declare_parameters();
        fs::create_directories(profile_dir_);

        init_detector();
        init_sface_engine();
        load_profiles();

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10,
            std::bind(&JupiterFaceRecognition::image_callback, this, std::placeholders::_1));

        trigger_sub_ = create_subscription<std_msgs::msg::String>(
            "/vision/trigger", 10,
            std::bind(&JupiterFaceRecognition::trigger_callback, this, std::placeholders::_1));

        register_sub_ = create_subscription<std_msgs::msg::String>(
            "/vision/register", 10,
            std::bind(&JupiterFaceRecognition::register_callback, this, std::placeholders::_1));

        user_pub_   = create_publisher<std_msgs::msg::String>("/current_user", 10);
        status_pub_ = create_publisher<std_msgs::msg::String>("/vision/status", 10);

        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                std_msgs::msg::String msg;
                msg.data = stable_user_;
                user_pub_->publish(msg);
            });

        RCLCPP_INFO(get_logger(), "Jupiter Face Recognition online");
        RCLCPP_INFO(get_logger(), "Profiles: %s | Known users: %zu",
            profile_dir_.c_str(), profiles_.size());
    }

private:
    // ── Parameters ────────────────────────────────────────────────────────────

    void declare_parameters() {
        const char* home = std::getenv("HOME");
        const std::string home_str = home ? home : "/home/jupiter";

        declare_parameter("detector_model",
            home_str + "/jupitercpp_ws/models/face_detection_yunet_2023mar.onnx");
        declare_parameter("recognizer_model",
            home_str + "/jupitercpp_ws/models/face_recognition_sface_2021dec.onnx");
        declare_parameter("profile_dir",
            home_str + "/jupitercpp_ws/memory/profiles");
        declare_parameter("snapshot_path",   std::string("/tmp/jupiter_view.jpg"));
        declare_parameter("match_threshold", 0.40);    // cosine similarity threshold (L2-normed)
        declare_parameter("unknown_streak",  30);
        declare_parameter("guest_streak",    5);
        declare_parameter("input_width",     640);
        declare_parameter("input_height",    480);

        detector_model_     = get_parameter("detector_model").as_string();
        recognizer_model_   = get_parameter("recognizer_model").as_string();
        profile_dir_        = get_parameter("profile_dir").as_string();
        snapshot_path_      = get_parameter("snapshot_path").as_string();
        match_threshold_    = get_parameter("match_threshold").as_double();
        unknown_streak_max_ = get_parameter("unknown_streak").as_int();
        guest_streak_max_   = get_parameter("guest_streak").as_int();
        input_width_        = get_parameter("input_width").as_int();
        input_height_       = get_parameter("input_height").as_int();
    }

    // ── Model init ────────────────────────────────────────────────────────────

    void init_detector() {
        // YuNet runs on OpenCV CPU — CUDA DNN backend not available in system build
        detector_ = cv::FaceDetectorYN::create(
            detector_model_, "",
            cv::Size(input_width_, input_height_),
            0.9f,   // score threshold
            0.3f,   // NMS threshold
            5000);  // top-k before NMS

        RCLCPP_INFO(get_logger(), "YuNet detector loaded (CPU)");
    }

    void init_sface_engine() {
        // SFace alignment helper — use OpenCV recognizer solely for alignCrop()
        aligner_ = cv::FaceRecognizerSF::create(recognizer_model_, "");

        sface_ = std::make_unique<SFaceEngine>(recognizer_model_);
        if (sface_->valid()) {
            RCLCPP_INFO(get_logger(), "SFace embedding engine loaded on CUDA (TensorRT)");
        } else {
            RCLCPP_ERROR(get_logger(), "SFace TRT engine failed — recognition unavailable");
        }
    }

    // ── Profile management ────────────────────────────────────────────────────

    void load_profiles() {
        std::lock_guard<std::mutex> lock(profiles_mutex_);
        profiles_.clear();

        for (const auto& entry : fs::directory_iterator(profile_dir_)) {
            if (entry.path().extension() != ".json") continue;

            std::ifstream f(entry.path());
            if (!f.is_open()) continue;

            try {
                const json data = json::parse(f);
                if (!data.contains("embedding") || data["embedding"].empty()) continue;

                UserProfile profile;
                profile.name      = data["name"].get<std::string>();
                profile.embedding = data["embedding"].get<std::vector<float>>();
                profiles_.push_back(std::move(profile));
                RCLCPP_INFO(get_logger(), "Loaded profile: %s", profiles_.back().name.c_str());
            } catch (const json::exception& e) {
                RCLCPP_WARN(get_logger(), "Skipping bad profile %s: %s",
                    entry.path().c_str(), e.what());
            }
        }
    }

    void save_profile(const std::string& name, const std::vector<float>& embedding) {
        const std::string path = profile_dir_ + "/" + name + ".json";
        json data;
        data["name"]      = name;
        data["embedding"] = embedding;
        data["joined"]    = std::to_string(std::time(nullptr));

        std::ofstream f(path);
        f << data.dump(4);

        std::lock_guard<std::mutex> lock(profiles_mutex_);
        for (auto& p : profiles_) {
            if (p.name == name) { p.embedding = embedding; return; }
        }
        profiles_.push_back({name, embedding});
        RCLCPP_INFO(get_logger(), "Profile saved: %s", name.c_str());
    }

    // ── Recognition ───────────────────────────────────────────────────────────

    // Cosine similarity of two L2-normalised vectors
    static float cosine_sim(const std::vector<float>& a, const std::vector<float>& b) {
        float dot = 0.0f;
        const size_t n = std::min(a.size(), b.size());
        for (size_t i = 0; i < n; ++i) dot += a[i] * b[i];
        return dot;   // both L2-normalised, so dot == cosine similarity
    }

    std::string identify_face(const cv::Mat& frame, cv::Rect& out_rect) {
        cv::Mat faces;
        detector_->setInputSize(frame.size());
        detector_->detect(frame, faces);

        if (faces.rows == 0) return {};

        out_rect = cv::Rect(
            static_cast<int>(faces.at<float>(0, 0)),
            static_cast<int>(faces.at<float>(0, 1)),
            static_cast<int>(faces.at<float>(0, 2)),
            static_cast<int>(faces.at<float>(0, 3)));

        // Align the highest-confidence face
        cv::Mat aligned;
        aligner_->alignCrop(frame, faces.row(0), aligned);

        const auto embedding = sface_->embed(aligned);
        if (embedding.empty()) return "Guest";

        std::lock_guard<std::mutex> lock(profiles_mutex_);
        std::string best_name  = "Guest";   // face detected but no profile match → Guest
        float       best_score = static_cast<float>(match_threshold_);

        for (const auto& profile : profiles_) {
            const float score = cosine_sim(embedding, profile.embedding);
            RCLCPP_DEBUG(get_logger(), "[MATCH] %s similarity=%.4f threshold=%.2f",
                         profile.name.c_str(), score, match_threshold_);
            if (score > best_score) {
                best_score = score;
                best_name  = profile.name;
            }
        }

        if (best_name != "Guest") {
            RCLCPP_INFO(get_logger(), "[MATCH] Matched %s score=%.4f", best_name.c_str(), best_score);
        } else {
            // Log the closest miss so we can tune the threshold
            float closest = 0.0f;
            std::string closest_name = "none";
            for (const auto& profile : profiles_) {
                const float score = cosine_sim(embedding, profile.embedding);
                if (score > closest) { closest = score; closest_name = profile.name; }
            }
            RCLCPP_INFO(get_logger(), "[MATCH] Guest (closest: %s=%.4f, threshold=%.2f)",
                        closest_name.c_str(), closest, match_threshold_);
        }

        return best_name;
    }

    std::vector<float> embed_face(const cv::Mat& frame, const cv::Mat& face_row) {
        cv::Mat aligned;
        aligner_->alignCrop(frame, face_row, aligned);
        return sface_->embed(aligned);
    }

    // ── ROS2 callbacks ────────────────────────────────────────────────────────

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
            return;
        }

        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            latest_frame_ = frame.clone();
        }

        if (++frame_counter_ % recognition_interval_ != 0) return;

        cv::Rect detected_rect;
        const std::string detected = identify_face(frame, detected_rect);
        if (!detected.empty()) {
            std::lock_guard<std::mutex> lk(bbox_mutex_);
            last_face_rect_ = detected_rect;
        }

        if (detected.empty()) {
            // No face in frame at all
            guest_streak_count_ = 0;
            unknown_streak_count_++;
            if (unknown_streak_count_ >= unknown_streak_max_) stable_user_ = "Unknown";
            return;
        }

        if (detected == "Guest") {
            // Face present but not in any profile
            unknown_streak_count_ = 0;
            guest_streak_count_++;
            if (guest_streak_count_ >= guest_streak_max_ && stable_user_ != "Guest") {
                stable_user_ = "Guest";
                guest_streak_count_ = 0;
                RCLCPP_INFO(get_logger(), "[FACE] Unknown person — greeting as Guest");
                std_msgs::msg::String status_msg;
                status_msg.data = "detected:Guest";
                status_pub_->publish(status_msg);
            }
        } else {
            // Known registered user
            unknown_streak_count_ = 0;
            guest_streak_count_ = 0;
            if (stable_user_ != detected) {
                stable_user_ = detected;
                RCLCPP_INFO(get_logger(), "[FACE] Identified: %s", detected.c_str());
                std_msgs::msg::String status_msg;
                status_msg.data = "identified:" + detected;
                status_pub_->publish(status_msg);
            }
        }
    }

    void trigger_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data != "capture") return;

        std::lock_guard<std::mutex> lock(frame_mutex_);
        if (latest_frame_.empty()) {
            RCLCPP_WARN(get_logger(), "Snapshot requested but no frame available");
            return;
        }
        // Save the raw frame — no annotation. Identity is provided to the VLM
        // via the text bridge (for_user parameter in brain.cpp), so the
        // face bbox+label is not needed and was confusing llava into thinking
        // the label was part of a phone app UI.
        cv::imwrite(snapshot_path_, latest_frame_);
        RCLCPP_INFO(get_logger(), "Snapshot saved: %s", snapshot_path_.c_str());
    }

    void register_callback(const std_msgs::msg::String::SharedPtr msg) {
        const std::string name = msg->data;
        if (name.empty()) return;

        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (latest_frame_.empty()) {
                RCLCPP_WARN(get_logger(), "Register requested but no frame available");
                return;
            }
            frame = latest_frame_.clone();
        }

        cv::Mat faces;
        detector_->setInputSize(frame.size());
        detector_->detect(frame, faces);

        if (faces.rows == 0) {
            RCLCPP_WARN(get_logger(), "Registration failed — no face detected for %s", name.c_str());
            std_msgs::msg::String status;
            status.data = "register_failed:no_face";
            status_pub_->publish(status);
            return;
        }

        const auto embedding = embed_face(frame, faces.row(0));
        if (embedding.empty()) {
            RCLCPP_ERROR(get_logger(), "Registration failed — embedding error for %s", name.c_str());
            return;
        }

        cv::imwrite(profile_dir_ + "/" + name + ".jpg", frame);
        save_profile(name, embedding);

        stable_user_ = name;

        std_msgs::msg::String status;
        status.data = "registered:" + name;
        status_pub_->publish(status);
        RCLCPP_INFO(get_logger(), "Registered new user: %s", name.c_str());
    }

    // ── Members ───────────────────────────────────────────────────────────────

    std::string detector_model_;
    std::string recognizer_model_;
    std::string profile_dir_;
    std::string snapshot_path_;
    double      match_threshold_{0.40};
    int         unknown_streak_max_{30};
    int         input_width_{640};
    int         input_height_{480};

    cv::Ptr<cv::FaceDetectorYN>   detector_;
    cv::Ptr<cv::FaceRecognizerSF> aligner_;    // used only for alignCrop()
    std::unique_ptr<SFaceEngine>  sface_;

    std::vector<UserProfile> profiles_;
        std::mutex               profiles_mutex_;

    cv::Mat    latest_frame_;
    std::mutex frame_mutex_;
    cv::Rect   last_face_rect_;
    std::mutex bbox_mutex_;
    int        frame_counter_{0};
    const int  recognition_interval_{15};

    std::string stable_user_{"Unknown"};
    int         unknown_streak_count_{0};
    int         guest_streak_count_{0};
    int         guest_streak_max_{5};

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr   trigger_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr   register_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr      user_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr      status_pub_;
    rclcpp::TimerBase::SharedPtr                             timer_;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JupiterFaceRecognition>());
    rclcpp::shutdown();
    return 0;
}
