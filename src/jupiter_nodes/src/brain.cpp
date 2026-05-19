// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// Jupiter Brain Node
// /voice/raw_text      → intent detect → text LLM (gemma4:e2b) or vision LLM (llava:7b)
// /current_user        → tracks who Jupiter is speaking with
// /vision/trigger      → captures snapshot for VLM queries
// /vision/register     → triggers face registration when user consents
//
// Visual intent keywords route to llava:7b with the current camera frame.
// Conversation history is saved per user to disk.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <curl/curl.h>
#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using json = nlohmann::json;
namespace fs = std::filesystem;

namespace {

size_t curl_write_cb(char* ptr, size_t size, size_t nmemb, std::string* data) {
    data->append(ptr, size * nmemb);
    return size * nmemb;
}

std::string strip_non_ascii(const std::string& text) {
    std::string out;
    out.reserve(text.size());
    for (unsigned char c : text) {
        if (c >= 0x20 && c <= 0x7E) out.push_back(static_cast<char>(c));
    }
    std::string result;
    result.reserve(out.size());
    bool prev_space = false;
    for (char c : out) {
        if (c == ' ') {
            if (!prev_space) result.push_back(c);
            prev_space = true;
        } else {
            result.push_back(c);
            prev_space = false;
        }
    }
    return result;
}

std::string current_timestamp() {
    const std::time_t now = std::time(nullptr);
    char buf[64];
    std::strftime(buf, sizeof(buf), "%A %Y-%m-%d %H:%M:%S", std::localtime(&now));
    return buf;
}

// ── Base64 encoder (for JPEG → VLM) ──────────────────────────────────────────

static const char* kBase64Chars =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

std::string base64_encode(const std::vector<uint8_t>& data) {
    std::string result;
    result.reserve((data.size() + 2) / 3 * 4);
    size_t i = 0;
    const size_t n = data.size();
    while (i < n) {
        const uint32_t b0 = data[i++];
        const uint32_t b1 = i < n ? data[i++] : 0;
        const uint32_t b2 = i < n ? data[i++] : 0;
        result.push_back(kBase64Chars[(b0 >> 2) & 0x3F]);
        result.push_back(kBase64Chars[((b0 << 4) | (b1 >> 4)) & 0x3F]);
        result.push_back(kBase64Chars[((b1 << 2) | (b2 >> 6)) & 0x3F]);
        result.push_back(kBase64Chars[b2 & 0x3F]);
    }
    // Padding
    const size_t pad = (3 - n % 3) % 3;
    for (size_t p = 0; p < pad; ++p) result[result.size() - 1 - p] = '=';
    return result;
}

// Visual intent keywords — any match routes to the VLM
bool is_visual_query(const std::string& text) {
    std::string lower = text;
    for (char& c : lower) c = static_cast<char>(std::tolower(c));

    static const std::vector<std::string> kKeywords = {
        "what do you see",  "what can you see",  "what are you seeing",
        "look at",          "describe what",      "describe the",
        "what's in front",  "what is in front",   "what's around",
        "what's behind",    "what am i holding",  "what is that",
        "what is this",     "can you see",        "what's on the",
        "what is on the",   "read that",          "read this",
        "what does it say", "what's happening",   "scan the room",
        "look around",      "take a look",        "what do i look like",
    };
    for (const auto& kw : kKeywords) {
        if (lower.find(kw) != std::string::npos) return true;
    }
    return false;
}

} // namespace


class JupiterBrain : public rclcpp::Node {
public:
    explicit JupiterBrain() : Node("jupiter_brain") {
        declare_parameters();
        curl_global_init(CURL_GLOBAL_ALL);
        fs::create_directories(memory_dir_);

        response_pub_  = create_publisher<std_msgs::msg::String>("/voice/response_text", 10);
        register_pub_  = create_publisher<std_msgs::msg::String>("/vision/register", 10);
        trigger_pub_   = create_publisher<std_msgs::msg::String>("/vision/trigger", 10);

        text_sub_ = create_subscription<std_msgs::msg::String>(
            "/voice/raw_text", 10,
            std::bind(&JupiterBrain::text_callback, this, std::placeholders::_1));

        user_sub_ = create_subscription<std_msgs::msg::String>(
            "/current_user", 10,
            std::bind(&JupiterBrain::user_callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Jupiter Brain online — text: %s | vision: %s",
            llama_url_.c_str(), vision_model_.c_str());
        RCLCPP_INFO(get_logger(), "Memory dir: %s", memory_dir_.c_str());
    }

    ~JupiterBrain() {
        if (llm_thread_.joinable()) llm_thread_.join();
        curl_global_cleanup();
    }

private:
    // ── Parameters ────────────────────────────────────────────────────────────

    void declare_parameters() {
        const char* home = std::getenv("HOME");
        const std::string home_str = home ? home : "/home/jupiter";

        declare_parameter("llama_url",         std::string("http://localhost:11434"));
        declare_parameter("model_name",        std::string("gemma4:e2b"));
        declare_parameter("vision_model",      std::string("llava:7b"));
        declare_parameter("snapshot_path",     std::string("/tmp/jupiter_view.jpg"));
        declare_parameter("memory_dir",        home_str + "/jupitercpp_ws/memory/conversations");
        declare_parameter("system_prompt",     std::string(
            "You are Jupiter, a friendly and helpful home robot assistant. "
            "Your responses are spoken aloud, so keep them to 1-3 sentences — "
            "concise, natural, and conversational. "
            "You know the people who live here and greet them by name. "
            "If you are speaking with a guest you do not recognise, be warm and welcoming."));
        declare_parameter("max_tokens",        400);
        declare_parameter("temperature",       0.7);
        declare_parameter("max_history_turns", 10);

        llama_url_         = get_parameter("llama_url").as_string();
        model_name_        = get_parameter("model_name").as_string();
        vision_model_      = get_parameter("vision_model").as_string();
        snapshot_path_     = get_parameter("snapshot_path").as_string();
        memory_dir_        = get_parameter("memory_dir").as_string();
        system_prompt_     = get_parameter("system_prompt").as_string();
        max_tokens_        = get_parameter("max_tokens").as_int();
        temperature_       = get_parameter("temperature").as_double();
        max_history_turns_ = get_parameter("max_history_turns").as_int();
    }

    // ── User tracking ─────────────────────────────────────────────────────────

    void user_callback(const std_msgs::msg::String::SharedPtr msg) {
        const std::string incoming = msg->data;
        if (incoming == current_user_) return;

        const std::string previous = current_user_;
        current_user_ = incoming;

        RCLCPP_INFO(get_logger(), "[USER CHANGE] %s → %s", previous.c_str(), incoming.c_str());

        if (!previous.empty()) save_history(previous);
        load_history(current_user_);

        if (incoming != "Unknown" || previous != "Unknown") {
            greet_user(incoming, previous);
        }
    }

    void greet_user(const std::string& user, const std::string& previous) {
        if (user == "Unknown") {
            save_history(previous);
            history_.clear();
            pending_registration_name_.clear();
            awaiting_registration_consent_ = false;
            return;
        }

        std::string greeting;
        if (user == "Guest") {
            greeting = "Hello! I am Jupiter. I do not think we have met before. "
                       "Would you like me to remember you? If so, just tell me your name.";
            awaiting_registration_consent_ = true;
        } else if (history_.empty()) {
            greeting = "Hello " + user + "! Good to see you. How can I help?";
        } else {
            greeting = "Welcome back, " + user + "! What can I do for you?";
        }

        speak(greeting);
    }

    // ── Per-user conversation persistence ─────────────────────────────────────

    void load_history(const std::string& user) {
        std::lock_guard<std::mutex> lock(history_mutex_);
        history_.clear();

        if (user == "Unknown" || user == "Guest") return;

        const fs::path path = fs::path(memory_dir_) / (user + ".json");
        if (!fs::exists(path)) return;

        std::ifstream f(path);
        if (!f.is_open()) return;

        try {
            const json data = json::parse(f);
            if (!data.contains("history")) return;
            for (const auto& entry : data["history"]) {
                history_.push_back({entry["role"].get<std::string>(),
                                    entry["content"].get<std::string>()});
            }
            while (static_cast<int>(history_.size()) > max_history_turns_ * 2) {
                history_.erase(history_.begin());
            }
            RCLCPP_INFO(get_logger(), "Loaded %zu history messages for %s",
                history_.size(), user.c_str());
        } catch (const json::exception& e) {
            RCLCPP_WARN(get_logger(), "Failed to load history for %s: %s", user.c_str(), e.what());
            history_.clear();
        }
    }

    void save_history(const std::string& user) {
        if (user == "Unknown" || user == "Guest" || user.empty()) return;

        std::lock_guard<std::mutex> lock(history_mutex_);
        if (history_.empty()) return;

        fs::create_directories(memory_dir_);
        const fs::path path = fs::path(memory_dir_) / (user + ".json");
        json data;
        data["user"]      = user;
        data["last_seen"] = std::to_string(std::time(nullptr));
        data["history"]   = json::array();
        for (const auto& m : history_) {
            data["history"].push_back({{"role", m.role}, {"content", m.content}});
        }
        std::ofstream f(path);
        f << data.dump(4);
        RCLCPP_INFO(get_logger(), "Saved %zu history messages for %s",
            history_.size(), user.c_str());
    }

    // ── ROS2 text callback ────────────────────────────────────────────────────

    void text_callback(const std_msgs::msg::String::SharedPtr msg) {
        const std::string user_text = msg->data;
        if (user_text.empty()) return;

        RCLCPP_INFO(get_logger(), "[USER/%s] %s", current_user_.c_str(), user_text.c_str());

        if (awaiting_registration_consent_) {
            handle_registration_flow(user_text);
            return;
        }

        // Drop incoming query if LLM is already busy — one request at a time
        if (llm_busy_.exchange(true)) {
            RCLCPP_WARN(get_logger(), "LLM busy — ignoring query");
            return;
        }

        // Run the LLM call on a background thread so the spin loop stays responsive
        if (llm_thread_.joinable()) llm_thread_.join();
        const std::string captured_user = current_user_;
        const bool visual = is_visual_query(user_text);
        llm_thread_ = std::thread([this, user_text, captured_user, visual]() {
            std::string reply;
            if (visual) {
                RCLCPP_INFO(get_logger(), "[VISION] Visual query detected — capturing frame");
                reply = strip_non_ascii(query_vlm(user_text));
            } else {
                reply = strip_non_ascii(query_llm(user_text));
            }
            llm_busy_.store(false);

            if (reply.empty()) {
                RCLCPP_WARN(get_logger(), "Empty LLM response");
                return;
            }

            RCLCPP_INFO(get_logger(), "[JUPITER] %s", reply.c_str());
            update_history(user_text, reply);
            speak(reply);
        });
        llm_thread_.detach();
    }

    // ── VLM query (llava:7b) ──────────────────────────────────────────────────

    std::string query_vlm(const std::string& user_text) {
        // Trigger snapshot capture and wait for it to land
        std_msgs::msg::String trigger;
        trigger.data = "capture";
        trigger_pub_->publish(trigger);
        std::this_thread::sleep_for(std::chrono::milliseconds(400));

        // Read and base64-encode the snapshot
        const std::string image_b64 = load_image_base64(snapshot_path_);
        if (image_b64.empty()) {
            RCLCPP_WARN(get_logger(), "No snapshot available — falling back to text LLM");
            return query_llm(user_text);
        }

        const std::string vision_system =
            "You are Jupiter, a home robot. You are looking at what your camera sees right now. "
            "Describe what you see concisely in 1-3 sentences, spoken naturally. "
            "Be specific about objects, people, text, and the environment.";

        // Build OpenAI multimodal message — content is an array with text + image
        json user_content = json::array();
        user_content.push_back({{"type", "text"}, {"text", user_text}});
        user_content.push_back({
            {"type", "image_url"},
            {"image_url", {{"url", "data:image/jpeg;base64," + image_b64}}}
        });

        json messages = json::array();
        messages.push_back({{"role", "system"}, {"content", vision_system}});
        messages.push_back({{"role", "user"},   {"content", user_content}});

        const json request_body = {
            {"model",       vision_model_},
            {"messages",    messages},
            {"max_tokens",  300},
            {"temperature", 0.3},
            {"stream",      false}
        };

        RCLCPP_INFO(get_logger(), "[VISION] Querying %s with image (%zu B b64)",
            vision_model_.c_str(), image_b64.size());

        return http_post(llama_url_ + "/v1/chat/completions", request_body.dump(), 90L);
    }

    std::string load_image_base64(const std::string& path) {
        std::ifstream f(path, std::ios::binary | std::ios::ate);
        if (!f.is_open()) return {};

        const std::streamsize size = f.tellg();
        if (size <= 0) return {};

        f.seekg(0, std::ios::beg);
        std::vector<uint8_t> buf(size);
        if (!f.read(reinterpret_cast<char*>(buf.data()), size)) return {};

        return base64_encode(buf);
    }

    // ── Text LLM query (gemma4:e2b) ───────────────────────────────────────────

    std::string query_llm(const std::string& user_text) {
        const std::string timestamp = current_timestamp();

        std::string contextual_system = system_prompt_;
        if (current_user_ != "Unknown" && current_user_ != "Guest") {
            contextual_system += " You are speaking with " + current_user_ + ".";
        }
        contextual_system += " The current date and time is: " + timestamp + ".";

        json messages = json::array();
        messages.push_back({{"role", "system"}, {"content", contextual_system}});

        {
            std::lock_guard<std::mutex> lock(history_mutex_);
            for (const auto& m : history_) {
                messages.push_back({{"role", m.role}, {"content", m.content}});
            }
        }

        messages.push_back({{"role", "user"},
            {"content", "[" + timestamp + "] " + user_text}});

        const json request_body = {
            {"model",       model_name_},
            {"messages",    messages},
            {"max_tokens",  max_tokens_},
            {"temperature", temperature_},
            {"stream",      false}
        };

        return http_post(llama_url_ + "/v1/chat/completions", request_body.dump(), 60L);
    }

    // ── Shared HTTP POST → extracts assistant content ─────────────────────────

    std::string http_post(const std::string& url, const std::string& body, long timeout_s) {
        std::string response_raw;

        CURL* curl = curl_easy_init();
        if (!curl) {
            RCLCPP_ERROR(get_logger(), "curl_easy_init() failed");
            return {};
        }

        struct curl_slist* headers = nullptr;
        headers = curl_slist_append(headers, "Content-Type: application/json");

        curl_easy_setopt(curl, CURLOPT_URL,          url.c_str());
        curl_easy_setopt(curl, CURLOPT_POST,          1L);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS,    body.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(body.size()));
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER,    headers);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_write_cb);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA,     &response_raw);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT,       timeout_s);

        const CURLcode rc = curl_easy_perform(curl);
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);

        if (rc != CURLE_OK) {
            RCLCPP_ERROR(get_logger(), "HTTP request failed: %s", curl_easy_strerror(rc));
            return {};
        }

        try {
            const json resp = json::parse(response_raw);
            const json& msg = resp.at("choices").at(0).at("message");
            if (msg.contains("content") && !msg.at("content").is_null()) {
                return msg.at("content").get<std::string>();
            }
            RCLCPP_ERROR(get_logger(), "content null | raw: %.200s", response_raw.c_str());
            return {};
        } catch (const json::exception& e) {
            RCLCPP_ERROR(get_logger(), "JSON parse error: %s | raw: %.200s",
                e.what(), response_raw.c_str());
            return {};
        }
    }

    // ── History management ────────────────────────────────────────────────────

    void update_history(const std::string& user_text, const std::string& reply) {
        std::lock_guard<std::mutex> lock(history_mutex_);
        history_.push_back({"user",      user_text});
        history_.push_back({"assistant", reply});
        while (static_cast<int>(history_.size()) > max_history_turns_ * 2) {
            history_.erase(history_.begin());
        }
        if (current_user_ == "Unknown" || current_user_ == "Guest") return;

        // Auto-save after every exchange
        fs::create_directories(memory_dir_);
        const fs::path path = fs::path(memory_dir_) / (current_user_ + ".json");
        json data;
        data["user"]      = current_user_;
        data["last_seen"] = std::to_string(std::time(nullptr));
        data["history"]   = json::array();
        for (const auto& m : history_) {
            data["history"].push_back({{"role", m.role}, {"content", m.content}});
        }
        std::ofstream f(path);
        f << data.dump(4);
    }

    // ── Guest registration flow ───────────────────────────────────────────────

    void handle_registration_flow(const std::string& user_text) {
        const std::string lower = [&]() {
            std::string s = user_text;
            for (char& c : s) c = static_cast<char>(std::tolower(c));
            return s;
        }();

        if (lower.find("no") != std::string::npos ||
            lower.find("nope") != std::string::npos ||
            lower.find("not now") != std::string::npos) {
            awaiting_registration_consent_ = false;
            speak("No problem! I will call you Guest. How can I help you today?");
            return;
        }

        std::string name;
        for (char c : user_text) {
            if (std::isalpha(c) || c == ' ') name.push_back(c);
        }
        const auto first = name.find_first_not_of(' ');
        if (first == std::string::npos) {
            speak("Sorry, I did not catch your name. Could you say it again?");
            return;
        }
        name = name.substr(first);
        const auto space = name.find(' ');
        if (space != std::string::npos) name = name.substr(0, space);
        if (!name.empty()) name[0] = static_cast<char>(std::toupper(name[0]));

        speak("Nice to meet you, " + name + "! Let me take a look at you to remember your face.");

        std_msgs::msg::String reg_msg;
        reg_msg.data = name;
        register_pub_->publish(reg_msg);

        awaiting_registration_consent_ = false;
        speak("Done! I will remember you as " + name + " from now on. How can I help?");
    }

    void speak(const std::string& text) {
        std_msgs::msg::String out;
        out.data = strip_non_ascii(text);
        response_pub_->publish(out);
    }

    // ── Members ───────────────────────────────────────────────────────────────

    std::string llama_url_;
    std::string model_name_;
    std::string vision_model_;
    std::string snapshot_path_;
    std::string memory_dir_;
    std::string system_prompt_;
    int         max_tokens_{400};
    double      temperature_{0.7};
    int         max_history_turns_{10};

    std::string current_user_{"Unknown"};
    bool        awaiting_registration_consent_{false};
    std::string pending_registration_name_;

    std::atomic<bool> llm_busy_{false};
    std::thread       llm_thread_;

    struct Message { std::string role; std::string content; };
    std::vector<Message> history_;
    std::mutex           history_mutex_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    response_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    register_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    trigger_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr text_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr user_sub_;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JupiterBrain>());
    rclcpp::shutdown();
    return 0;
}
