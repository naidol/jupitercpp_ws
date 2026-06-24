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
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

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
    std::string result;
    result.reserve(text.size());
    
    bool prev_space = false;
    for (unsigned char c : text) {
        if (c >= 0x20 && c <= 0x7E) {
            if (c == ' ') {
                if (!prev_space) result.push_back(c);
                prev_space = true;
            } else {
                result.push_back(static_cast<char>(c));
                prev_space = false;
            }
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

bool is_sleep_phrase(const std::string& text) {
    std::string lower = text;
    for (char& c : lower) c = static_cast<char>(std::tolower(c));
    static const std::vector<std::string> kPhrases = {
        "go to sleep", "jupiter sleep", "goodnight jupiter", "sleep mode"
    };
    for (const auto& p : kPhrases) {
        if (lower.find(p) != std::string::npos) return true;
    }
    return false;
}

bool is_dock_phrase(const std::string& text) {
    std::string lower = text;
    for (char& c : lower) c = static_cast<char>(std::tolower(c));
    static const std::vector<std::string> kPhrases = {
        "go to the dock", "go to dock", "go to your dock", "go and dock",
        "dock yourself", "go charge", "go and charge", "return to dock"
    };
    for (const auto& p : kPhrases) {
        if (lower.find(p) != std::string::npos) return true;
    }
    return false;
}

bool is_wake_phrase(const std::string& text) {
    std::string lower = text;
    for (char& c : lower) c = static_cast<char>(std::tolower(c));
    static const std::vector<std::string> kPhrases = {
        "wake up", "hey jupiter", "jupiter wake up"
    };
    for (const auto& p : kPhrases) {
        if (lower.find(p) != std::string::npos) return true;
    }
    return false;
}

// Extract a first name from natural speech — strips common preambles and rejects
// function words so "My name is Logan" and "I am Logan" both yield "Logan".
std::string extract_name(const std::string& text) {
    std::string lower = text;
    for (char& c : lower) c = static_cast<char>(std::tolower(c));

    static const std::vector<std::string> kPrefixes = {
        "hi my name is ", "hi i am ", "hi i'm ",
        "my name is ", "my name's ", "i am ", "i'm ",
        "call me ", "it's ", "its ", "the name is ",
        "name is ", "this is ", "just "
    };

    size_t name_start = 0;
    for (const auto& prefix : kPrefixes) {
        const size_t pos = lower.find(prefix);
        if (pos != std::string::npos) {
            name_start = pos + prefix.size();
            break;
        }
    }

    // Extract first alphabetic word from name_start
    std::string name;
    for (size_t i = name_start; i < text.size(); ++i) {
        if (std::isalpha(static_cast<unsigned char>(text[i]))) {
            name.push_back(text[i]);
        } else if (!name.empty()) {
            break;
        }
    }

    if (name.empty()) return {};
    name[0] = static_cast<char>(std::toupper(name[0]));

    // Reject common function words that are not names
    static const std::vector<std::string> kNotNames = {
        "I", "My", "Me", "The", "A", "An", "It", "Its",
        "Hi", "Hello", "Hey", "Yes", "Ok", "Okay", "Sure", "Well", "Just"
    };
    for (const auto& w : kNotNames) {
        if (name == w) return {};
    }
    return name;
}

// Visual intent keywords — any match routes to the VLM
bool is_visual_query(const std::string& text) {
    std::string lower = text;
    for (char& c : lower) c = static_cast<char>(std::tolower(c));

    static const std::vector<std::string> kKeywords = {
        "what do you see",  "what can you see",  "what are you seeing",
        "what you see",     "see in the",        "in the room",
        "look at",          "describe what",      "describe the",
        "what's in front",  "what is in front",   "what's around",
        "what's behind",    "what am i holding",  "what is that",
        "what is this",     "can you see",        "what's on the",
        "what is on the",   "read that",          "read this",
        "what does it say", "what's happening",   "scan the room",
        "look around",      "take a look",        "what do i look like",
        "around you",       "see anything",       "what's there",
        "in front of you",  "show me",            "what colour",
        "what color",
    };
    for (const auto& kw : kKeywords) {
        if (lower.find(kw) != std::string::npos) return true;
    }
    return false;
}

// Register-intent: known user wants to register someone new standing in front of the camera.
bool is_register_command(const std::string& text) {
    std::string lower = text;
    for (char& c : lower) c = static_cast<char>(std::tolower(c));
    static const std::vector<std::string> kKeywords = {
        "register", "add a user", "add new user", "add user",
        "remember this person", "remember her", "remember him",
        "who is this", "who is she", "who is he",
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
        sleep_pub_     = create_publisher<std_msgs::msg::Bool>("/jupiter/sleeping", 10);
        dock_pub_      = create_publisher<std_msgs::msg::Bool>("/dock/engage", 10);
        // Tells the voice node to relax its 2-word minimum filter so a single-word
        // name (e.g. "Logan") is accepted during guest registration.
        expecting_name_pub_ = create_publisher<std_msgs::msg::Bool>("/jupiter/expecting_name", 10);

        text_sub_ = create_subscription<std_msgs::msg::String>(
            "/voice/raw_text", 10,
            std::bind(&JupiterBrain::text_callback, this, std::placeholders::_1));

        user_sub_ = create_subscription<std_msgs::msg::String>(
            "/current_user", 10,
            std::bind(&JupiterBrain::user_callback, this, std::placeholders::_1));

        battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
            "/battery/state", 10,
            std::bind(&JupiterBrain::battery_callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Jupiter Brain online — text: %s | vision: %s",
            llama_url_.c_str(), vision_model_.c_str());
        RCLCPP_INFO(get_logger(), "Memory dir: %s", memory_dir_.c_str());
    }

    ~JupiterBrain() {
        // Wait for any in-flight LLM response so history is complete before saving
        while (llm_busy_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        save_history(current_user_);
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

    void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        battery_percentage_ = msg->percentage * 100.0f;
        battery_voltage_    = msg->voltage;
    }

    void user_callback(const std_msgs::msg::String::SharedPtr msg) {
        const std::string incoming = msg->data;
        if (incoming == current_user_) return;

        const std::string previous = current_user_;
        current_user_ = incoming;

        RCLCPP_INFO(get_logger(), "[USER CHANGE] %s → %s", previous.c_str(), incoming.c_str());

        if (!previous.empty()) save_history(previous);
        load_history(current_user_);

        if (sleeping_) return;

        if (incoming != "Unknown" || previous != "Unknown") {
            greet_user(incoming, previous);
        }
    }

    void greet_user(const std::string& user, const std::string& previous) {
        if (user == "Unknown") {
            save_history(previous);
            history_.clear();
            pending_registration_name_.clear();
            reg_state_ = RegState::None;
            set_expecting_name(false);
            return;
        }

        std::string greeting;
        if (user == "Guest") {
            greeting = "Hello! I am Jupiter. I do not think we have met. "
                       "To register you, please tell me your name — "
                       "just say, my name is, and then your name.";
            reg_state_ = RegState::AwaitingName;
            set_expecting_name(true);
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

    // Atomic write: write to .tmp then rename — prevents partial-write corruption on Ctrl+C.
    // Caller must hold history_mutex_.
    void write_history_atomic(const std::string& user) {
        fs::create_directories(memory_dir_);
        const fs::path path = fs::path(memory_dir_) / (user + ".json");
        const fs::path tmp  = fs::path(memory_dir_) / (user + ".json.tmp");

        json data;
        data["user"]      = user;
        data["last_seen"] = std::to_string(std::time(nullptr));
        data["history"]   = json::array();
        for (const auto& m : history_) {
            data["history"].push_back({{"role", m.role}, {"content", m.content}});
        }
        { std::ofstream f(tmp); f << data.dump(4); }
        fs::rename(tmp, path);
    }

    void save_history(const std::string& user) {
        if (user == "Unknown" || user == "Guest" || user.empty()) return;

        std::lock_guard<std::mutex> lock(history_mutex_);
        if (history_.empty()) return;

        write_history_atomic(user);
        RCLCPP_INFO(get_logger(), "Saved %zu history messages for %s",
            history_.size(), user.c_str());
    }

    // ── ROS2 text callback ────────────────────────────────────────────────────

    void text_callback(const std_msgs::msg::String::SharedPtr msg) {
        const std::string user_text = msg->data;
        if (user_text.empty()) return;

        RCLCPP_INFO(get_logger(), "[USER/%s] %s", current_user_.c_str(), user_text.c_str());

        if (sleeping_) {
            if (is_wake_phrase(user_text)) {
                sleeping_.store(false);
                publish_sleep_state(false);
                RCLCPP_INFO(get_logger(), "Wake phrase detected — resuming");
                speak("I am awake. How can I help?");
            } else {
                // Ack the voice node so it clears is_speaking_ immediately rather
                // than waiting out the 90-second response timeout.
                std_msgs::msg::String ack;
                response_pub_->publish(ack);
            }
            return;
        }
        if (is_sleep_phrase(user_text)) {
            sleeping_.store(true);
            publish_sleep_state(true);
            RCLCPP_INFO(get_logger(), "Sleep phrase detected — entering sleep mode");
            speak("Goodnight. Say wake up or hey Jupiter when you need me.");
            return;
        }

        if (is_dock_phrase(user_text)) {
            std_msgs::msg::Bool dock_msg;
            dock_msg.data = true;
            dock_pub_->publish(dock_msg);
            RCLCPP_INFO(get_logger(), "Dock phrase detected — engaging docking");
            speak("Heading to the dock.");
            return;
        }

        if (reg_state_ != RegState::None) {
            handle_registration_flow(user_text);
            return;
        }

        // Known user explicitly asking to register a new person in front of the camera
        if (is_register_command(user_text) && current_user_ != "Unknown" && current_user_ != "Guest") {
            RCLCPP_INFO(get_logger(), "Register command from %s — prompting for new user", current_user_.c_str());
            reg_state_ = RegState::AwaitingName;
            set_expecting_name(true);
            speak("Sure! Please have the person stand clearly in front of me. "
                  "Then say, my name is, and then your name.");
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
                reply = strip_non_ascii(query_vlm(user_text, captured_user));
            } else {
                reply = strip_non_ascii(query_llm(user_text, captured_user));
            }
            llm_busy_.store(false);

            if (reply.empty()) {
                RCLCPP_WARN(get_logger(), "Empty LLM response");
                return;
            }

            RCLCPP_INFO(get_logger(), "[JUPITER] %s", reply.c_str());
            update_history(user_text, reply, visual);
            speak(reply);
        });
    }

    // ── VLM query (llava:7b) ──────────────────────────────────────────────────

    std::string query_vlm(const std::string& user_text, const std::string& for_user) {
        // Remove any stale snapshot so we can detect when the fresh one arrives.
        // Without this, a file left from a previous session (possibly showing
        // different room content) would be silently read if trigger_callback
        // returns early (e.g. latest_frame_ not yet populated at startup).
        fs::remove(snapshot_path_);

        // Ask face_recognition to capture the current frame
        std_msgs::msg::String trigger;
        trigger.data = "capture";
        trigger_pub_->publish(trigger);

        // Poll for the fresh snapshot (up to 2 s at 50 ms intervals)
        const auto snap_deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(2);
        bool snap_ready = false;
        while (std::chrono::steady_clock::now() < snap_deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            std::ifstream test(snapshot_path_, std::ios::binary);
            if (test.good() && test.peek() != std::ifstream::traits_type::eof()) {
                snap_ready = true;
                break;
            }
        }
        if (!snap_ready) {
            RCLCPP_WARN(get_logger(), "Snapshot not written within 2 s — falling back to text LLM");
            return query_llm(user_text, for_user);
        }

        // Read and base64-encode the snapshot
        const std::string image_b64 = load_image_base64(snapshot_path_);
        if (image_b64.empty()) {
            RCLCPP_WARN(get_logger(), "Snapshot unreadable — falling back to text LLM");
            return query_llm(user_text, for_user);
        }

        // Build OpenAI multimodal message.
        // Face recognition burns an "IDENTIFIED: <name>" banner into the snapshot
        // before this call, so the VLM reads the identity directly from the image.
        const std::string vision_system =
            "You are a visual sensor on a home robot. Describe what you see in the image "
            "concisely in 2-3 sentences. If the image contains an 'IDENTIFIED:' label, "
            "refer to that person by the name shown. Be specific about people, objects, "
            "text, and the environment.";

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
            {"stream",      false},
            {"keep_alive",  -1}
        };

        RCLCPP_INFO(get_logger(), "[VISION] Querying %s with image (%zu B b64)",
            vision_model_.c_str(), image_b64.size());

        const std::string vlm_raw = http_post(llama_url_ + "/v1/chat/completions",
                                              request_body.dump(), 90L);
        if (vlm_raw.empty()) return {};

        // llava:7b cannot recognise private faces — it always returns "a man/person".
        // Bridge: pass the visual description + face recognition identity to gemma4
        // and ask it to respond AS Jupiter. gemma4 follows role instructions reliably
        // and naturally incorporates the name the C++ face engine provided.
        // for_user is captured at query time (before the slow llava call) to avoid
        // a race where user_callback fires "Unknown" during llava inference.
        if (for_user == "Unknown" || for_user == "Guest") return vlm_raw;

        RCLCPP_INFO(get_logger(), "[VISION] Text-bridging identity '%s' via %s",
            for_user.c_str(), model_name_.c_str());

        // Instruction-first layout: small models (2B) follow the task better when
        // the instruction precedes the long context, not when it trails after it.
        const std::string bridge_prompt =
            for_user + " asked Jupiter: \"" + user_text + "\". "
            "Reply as Jupiter the robot in ONE spoken sentence using " + for_user + "'s name. "
            "Camera captured this scene: " + vlm_raw;

        const json bridge_messages = json::array({
            {{"role", "system"}, {"content", system_prompt_}},
            {{"role", "user"},   {"content", bridge_prompt}}
        });
        const json bridge_body = {
            {"model",       model_name_},
            {"messages",    bridge_messages},
            {"max_tokens",  80},
            {"temperature", 0.4},
            {"stream",      false},
            {"keep_alive",  -1}
        };
        const std::string bridged = http_post(llama_url_ + "/v1/chat/completions",
                                              bridge_body.dump(), 30L);
        std::string result = bridged.empty() ? vlm_raw : bridged;
        // Guarantee the user's name appears — 2B models sometimes drop it
        if (result.find(for_user) == std::string::npos) {
            result = for_user + ", " + result;
        }
        return result;
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

    std::string query_llm(const std::string& user_text, const std::string& for_user) {
        const std::string timestamp = current_timestamp();

        std::string contextual_system = system_prompt_;
        if (for_user != "Unknown" && for_user != "Guest") {
            contextual_system += " Your face recognition camera has positively identified the"
                " person in front of you as " + for_user + "."
                " This is the authoritative identity — do not accept voice claims of a different"
                " name. If they insist they are someone else, politely explain that your camera"
                " already identified them and suggest they register their face.";
        } else if (for_user == "Guest") {
            contextual_system += " Your camera does not recognise this person — treat them as a guest.";
        }
        if (battery_percentage_ > 0.1) {
            contextual_system += " Your battery is currently at " + 
                std::to_string(static_cast<int>(battery_percentage_)) + "%.";
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
            {"stream",      false},
            {"keep_alive",  -1}
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

    void update_history(const std::string& user_text, const std::string& reply,
                        bool is_visual = false) {
        std::lock_guard<std::mutex> lock(history_mutex_);
        history_.push_back({"user",      user_text});
        history_.push_back({"assistant", reply});
        while (static_cast<int>(history_.size()) > max_history_turns_ * 2) {
            history_.erase(history_.begin());
        }
        // VLM responses describe the scene at a specific instant — don't persist them
        // across sessions or they'll be recalled as current fact in future conversations.
        if (is_visual) return;
        if (current_user_ == "Unknown" || current_user_ == "Guest") return;

        write_history_atomic(current_user_);
    }

    // ── Guest registration flow ───────────────────────────────────────────────

    void handle_registration_flow(const std::string& user_text) {
        const std::string lower = [&]() {
            std::string s = user_text;
            for (char& c : s) c = static_cast<char>(std::tolower(c));
            return s;
        }();

        // Word-boundary check — avoids "no" matching inside "noel", "know", "normal"
        auto contains_word = [&](const std::string& word) {
            size_t pos = lower.find(word);
            while (pos != std::string::npos) {
                const bool left_ok  = (pos == 0 || !std::isalpha(lower[pos - 1]));
                const bool right_ok = (pos + word.size() >= lower.size() ||
                                       !std::isalpha(lower[pos + word.size()]));
                if (left_ok && right_ok) return true;
                pos = lower.find(word, pos + 1);
            }
            return false;
        };

        // ── Universal cancel — bail out of registration from ANY step and stay as Guest ──
        // "no" is intentionally NOT here (at confirmation it means "wrong name, re-ask"); these are
        // the explicit escape hatches so the user is never trapped in the YES/NO loop.
        if (contains_word("cancel") || contains_word("stop") || contains_word("skip") ||
            contains_word("guest") ||
            lower.find("never mind") != std::string::npos ||
            lower.find("nevermind")  != std::string::npos ||
            lower.find("forget it")  != std::string::npos ||
            lower.find("leave it")   != std::string::npos ||
            lower.find("don't register")  != std::string::npos ||
            lower.find("do not register") != std::string::npos ||
            lower.find("not interested")  != std::string::npos) {
            pending_registration_name_.clear();
            reg_state_ = RegState::None;
            set_expecting_name(false);
            current_user_ = "Guest";   // stops greet_user from immediately re-offering registration
            speak("Okay, cancelling that. I will continue as Guest. How can I help?");
            return;
        }

        // ── Awaiting the name ────────────────────────────────────────────────
        if (reg_state_ == RegState::AwaitingName) {
            // User declines to register
            if (contains_word("no") || contains_word("nope") || contains_word("not now") ||
                lower.find("don't want") != std::string::npos ||
                lower.find("do not want") != std::string::npos) {
                reg_state_ = RegState::None;
                set_expecting_name(false);
                speak("No problem! I will call you Guest. How can I help you today?");
                return;
            }

            const std::string name = extract_name(user_text);
            if (name.empty()) {
                // Stay in AwaitingName, filter stays relaxed — re-prompt.
                speak("Sorry, I did not catch your name. "
                      "Please say, my name is, and then your name.");
                return;
            }

            pending_registration_name_ = name;
            reg_state_ = RegState::AwaitingConfirmation;
            // Filter stays relaxed so single-word "yes"/"no" is accepted.
            speak("I heard " + name + ". Is that correct? Please say yes or no.");
            return;
        }

        // ── Awaiting yes/no confirmation of the heard name ───────────────────
        if (reg_state_ == RegState::AwaitingConfirmation) {
            if (contains_word("yes") || contains_word("yeah") || contains_word("yep") ||
                contains_word("correct") || contains_word("right")) {
                const std::string name = pending_registration_name_;
                speak("Great! Let me take a look at you to remember your face.");

                std_msgs::msg::String reg_msg;
                reg_msg.data = name;
                register_pub_->publish(reg_msg);

                reg_state_ = RegState::None;
                set_expecting_name(false);
                pending_registration_name_.clear();
                current_user_ = name;  // prevent user_callback double-greeting on next tick
                speak("Done! I will remember you as " + name + " from now on. How can I help?");
                return;
            }

            if (contains_word("no") || contains_word("nope") || contains_word("wrong")) {
                pending_registration_name_.clear();
                reg_state_ = RegState::AwaitingName;
                speak("Sorry about that. Please tell me your name again.");
                return;
            }

            // Unclear answer — re-ask the confirmation.
            speak("Please say yes or no. Is your name " + pending_registration_name_ + "?");
            return;
        }
    }

    void speak(const std::string& text) {
        std_msgs::msg::String out;
        out.data = strip_non_ascii(text);
        response_pub_->publish(out);
    }

    void publish_sleep_state(bool sleeping) {
        std_msgs::msg::Bool msg;
        msg.data = sleeping;
        sleep_pub_->publish(msg);
    }

    // Tell the voice node whether to relax its 2-word minimum filter.
    // Relaxed while we await a name or a yes/no confirmation, so single-word
    // answers ("Logan", "yes", "no") are not dropped as hallucinations.
    void set_expecting_name(bool expecting) {
        std_msgs::msg::Bool msg;
        msg.data = expecting;
        expecting_name_pub_->publish(msg);
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
    // Registration state machine: None → AwaitingName → AwaitingConfirmation → None
    enum class RegState { None, AwaitingName, AwaitingConfirmation };
    RegState    reg_state_{RegState::None};
    std::string pending_registration_name_;
    float       battery_percentage_{0.0f};
    float       battery_voltage_{0.0f};

    std::atomic<bool> sleeping_{false};
    std::atomic<bool> llm_busy_{false};
    std::thread       llm_thread_;

    struct Message { std::string role; std::string content; };
    std::vector<Message> history_;
    std::mutex           history_mutex_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    response_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    register_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    trigger_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr      sleep_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr      dock_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr      expecting_name_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr text_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr user_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JupiterBrain>());
    rclcpp::shutdown();
    return 0;
}
