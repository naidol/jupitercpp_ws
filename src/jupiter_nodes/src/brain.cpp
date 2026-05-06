// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// Jupiter Brain Node
// /voice/raw_text → llama-server HTTP (OpenAI-compatible) → /voice/response_text
// Requires llama-server running: llama-server --model gemma-2b.gguf --port 8080 -ngl 99

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <curl/curl.h>
#include <nlohmann/json.hpp>

#include <chrono>
#include <ctime>
#include <mutex>
#include <string>
#include <vector>

using json = nlohmann::json;

namespace {

size_t curl_write_cb(char* ptr, size_t size, size_t nmemb, std::string* data) {
    data->append(ptr, size * nmemb);
    return size * nmemb;
}

// Keep only printable ASCII — removes emojis and other Unicode that piper can't speak
std::string strip_non_ascii(const std::string& text) {
    std::string out;
    out.reserve(text.size());
    for (unsigned char c : text) {
        if (c >= 0x20 && c <= 0x7E) {
            out.push_back(static_cast<char>(c));
        }
    }
    // Collapse multiple spaces left by removed characters
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

} // namespace


class JupiterBrain : public rclcpp::Node {
public:
    explicit JupiterBrain() : Node("jupiter_brain") {
        declare_parameters();
        curl_global_init(CURL_GLOBAL_ALL);

        response_pub_ = create_publisher<std_msgs::msg::String>("/voice/response_text", 10);

        text_sub_ = create_subscription<std_msgs::msg::String>(
            "/voice/raw_text", 10,
            std::bind(&JupiterBrain::text_callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Jupiter Brain online — llama-server @ %s", llama_url_.c_str());
        RCLCPP_INFO(get_logger(), "Model context window: %d history turns", max_history_turns_);
    }

    ~JupiterBrain() {
        curl_global_cleanup();
    }

private:
    // ── Parameters ────────────────────────────────────────────────────────────

    void declare_parameters() {
        declare_parameter("llama_url",        std::string("http://localhost:11434"));
        declare_parameter("model_name",       std::string("gemma4:e2b"));
        declare_parameter("system_prompt",    std::string(
            "You are Jupiter, a helpful home robot assistant. "
            "You respond concisely and naturally because your words are spoken aloud. "
            "Keep replies to 1-3 sentences unless the user asks for more detail. "
            "You are friendly, polite, and knowledgeable about the home environment."));
        declare_parameter("max_tokens",       2000);
        declare_parameter("temperature",      0.7);
        declare_parameter("max_history_turns", 6);

        llama_url_         = get_parameter("llama_url").as_string();
        model_name_        = get_parameter("model_name").as_string();
        system_prompt_     = get_parameter("system_prompt").as_string();
        max_tokens_        = get_parameter("max_tokens").as_int();
        temperature_       = get_parameter("temperature").as_double();
        max_history_turns_ = get_parameter("max_history_turns").as_int();
    }

    // ── ROS2 callback ─────────────────────────────────────────────────────────

    void text_callback(const std_msgs::msg::String::SharedPtr msg) {
        const std::string user_text = msg->data;
        if (user_text.empty()) return;

        RCLCPP_INFO(get_logger(), "[USER] %s", user_text.c_str());

        const std::string reply = strip_non_ascii(query_llm(user_text));
        if (reply.empty()) {
            RCLCPP_WARN(get_logger(), "Empty LLM response — is llama-server running?");
            return;
        }

        RCLCPP_INFO(get_logger(), "[JUPITER] %s", reply.c_str());

        // Update rolling conversation history
        {
            std::lock_guard<std::mutex> lock(history_mutex_);
            history_.push_back({"user",      user_text});
            history_.push_back({"assistant", reply});
            // Each turn = 2 messages; trim oldest when window exceeded
            while (static_cast<int>(history_.size()) > max_history_turns_ * 2) {
                history_.erase(history_.begin());
            }
        }

        std_msgs::msg::String out;
        out.data = reply;
        response_pub_->publish(out);
    }

    // ── LLM query via llama-server HTTP API ───────────────────────────────────

    std::string query_llm(const std::string& user_text) {
        // Inject current date/time so the LLM can answer time-related questions
        const std::time_t now = std::time(nullptr);
        char time_buf[64];
        std::strftime(time_buf, sizeof(time_buf), "%A %Y-%m-%d %H:%M:%S", std::localtime(&now));
        const std::string timed_system = system_prompt_ +
            " The current date and time is: " + time_buf + ".";

        // Build OpenAI-compatible messages array
        json messages = json::array();
        messages.push_back({{"role", "system"}, {"content", timed_system}});

        {
            std::lock_guard<std::mutex> lock(history_mutex_);
            for (const auto& m : history_) {
                messages.push_back({{"role", m.role}, {"content", m.content}});
            }
        }
        messages.push_back({{"role", "user"}, {"content", user_text}});

        const json request_body = {
            {"model",       model_name_},
            {"messages",    messages},
            {"max_tokens",  max_tokens_},
            {"temperature", temperature_},
            {"stream",      false}
        };

        const std::string url  = llama_url_ + "/v1/chat/completions";
        const std::string body = request_body.dump();
        std::string response_raw;

        CURL* curl = curl_easy_init();
        if (!curl) {
            RCLCPP_ERROR(get_logger(), "curl_easy_init() failed");
            return {};
        }

        struct curl_slist* headers = nullptr;
        headers = curl_slist_append(headers, "Content-Type: application/json");

        curl_easy_setopt(curl, CURLOPT_URL,           url.c_str());
        curl_easy_setopt(curl, CURLOPT_POST,           1L);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS,     body.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER,     headers);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,  curl_write_cb);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA,      &response_raw);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT,        60L);

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
            std::string content = msg.at("content").get<std::string>();
            // Thinking models return empty content + separate reasoning field
            if (content.empty() && msg.contains("reasoning")) {
                content = msg.at("reasoning").get<std::string>();
                RCLCPP_WARN(get_logger(), "content empty — using reasoning field (increase max_tokens)");
            }
            return content;
        } catch (const json::exception& e) {
            RCLCPP_ERROR(get_logger(), "JSON parse error: %s | raw: %s",
                e.what(), response_raw.substr(0, 200).c_str());
            return {};
        }
    }

    // ── Members ───────────────────────────────────────────────────────────────

    // Parameters
    std::string llama_url_;
    std::string model_name_;
    std::string system_prompt_;
    int         max_tokens_{200};
    double      temperature_{0.7};
    int         max_history_turns_{6};

    // Conversation history (role + content pairs)
    struct Message { std::string role; std::string content; };
    std::vector<Message> history_;
    std::mutex           history_mutex_;

    // ROS2
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    response_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr text_sub_;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JupiterBrain>());
    rclcpp::shutdown();
    return 0;
}
