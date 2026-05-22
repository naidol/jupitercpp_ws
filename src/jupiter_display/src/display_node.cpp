// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// Jupiter Display Node
// Subscribes to voice, face recognition, and system topics to drive a Qt6 QML
// animated robot face on the onboard 7-inch display.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/empty.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QObject>
#include <QString>
#include <QTimer>
#include <QMetaObject>
#include <QScreen>
#include <QWindow>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>

// ── Display state ─────────────────────────────────────────────────────────────
static constexpr int STATE_LISTENING  = 0;
static constexpr int STATE_PROCESSING = 1;
static constexpr int STATE_SPEAKING   = 2;

static constexpr int MODE_INTERACTION = 0;
static constexpr int MODE_NAVIGATING  = 1;

// ── System helpers ────────────────────────────────────────────────────────────

// Scan sysfs thermal zones for one whose type contains type_substr (lowercase).
static std::string find_thermal_path(const std::string& type_substr) {
    for (int i = 0; i < 32; ++i) {
        const std::string type_path =
            "/sys/class/thermal/thermal_zone" + std::to_string(i) + "/type";
        std::ifstream tf(type_path);
        if (!tf) break;
        std::string type;
        std::getline(tf, type);
        std::transform(type.begin(), type.end(), type.begin(), ::tolower);
        if (type.find(type_substr) != std::string::npos)
            return "/sys/class/thermal/thermal_zone" + std::to_string(i) + "/temp";
    }
    return {};
}

static float read_thermal(const std::string& path) {
    if (path.empty()) return 0.0f;
    std::ifstream f(path);
    if (!f) return 0.0f;
    int millideg = 0;
    f >> millideg;
    return millideg / 1000.0f;
}

// Returns 0–100 signal quality, or -1 if /proc/net/wireless unavailable.
static int read_wifi_signal() {
    std::ifstream f("/proc/net/wireless");
    if (!f) return -1;
    std::string line;
    std::getline(f, line);  // header 1
    std::getline(f, line);  // header 2
    if (!std::getline(f, line)) return 0;
    const auto colon = line.find(':');
    if (colon == std::string::npos) return 0;
    std::istringstream iss(line.substr(colon + 1));
    int status = 0;
    float link = 0.0f;
    if (!(iss >> status >> link)) return 0;
    return static_cast<int>(std::min(100.0f, (link / 70.0f) * 100.0f));
}

// ── DisplayBridge ─────────────────────────────────────────────────────────────
// QObject that lives on the Qt main thread. ROS2 callbacks post safely via
// QMetaObject::invokeMethod with Qt::QueuedConnection.

class DisplayBridge : public QObject {
    Q_OBJECT

    Q_PROPERTY(int     state         READ state         NOTIFY stateChanged)
    Q_PROPERTY(QString currentUser   READ currentUser   NOTIFY currentUserChanged)
    Q_PROPERTY(QString lastUtterance READ lastUtterance NOTIFY lastUtteranceChanged)
    Q_PROPERTY(QString lastResponse  READ lastResponse  NOTIFY lastResponseChanged)

    Q_PROPERTY(float   batteryVoltage READ batteryVoltage NOTIFY batteryVoltageChanged)
    Q_PROPERTY(float   batteryPct     READ batteryPct     NOTIFY batteryPctChanged)
    Q_PROPERTY(float   cpuTemp        READ cpuTemp        NOTIFY cpuTempChanged)
    Q_PROPERTY(float   gpuTemp        READ gpuTemp        NOTIFY gpuTempChanged)
    Q_PROPERTY(int     wifiSignal     READ wifiSignal     NOTIFY wifiSignalChanged)
    Q_PROPERTY(bool    cameraOnline   READ cameraOnline   NOTIFY cameraOnlineChanged)
    Q_PROPERTY(bool    lidarOnline    READ lidarOnline    NOTIFY lidarOnlineChanged)
    Q_PROPERTY(bool    voiceOnline    READ voiceOnline    NOTIFY voiceOnlineChanged)
    Q_PROPERTY(float   linearVel      READ linearVel      NOTIFY linearVelChanged)
    Q_PROPERTY(int     robotMode      READ robotMode      NOTIFY robotModeChanged)
    Q_PROPERTY(bool    sleeping       READ sleeping       NOTIFY sleepingChanged)

public:
    explicit DisplayBridge(QObject* parent = nullptr) : QObject(parent) {}

    int     state()          const { return state_; }
    QString currentUser()    const { return currentUser_; }
    QString lastUtterance()  const { return lastUtterance_; }
    QString lastResponse()   const { return lastResponse_; }
    float   batteryVoltage() const { return battery_voltage_; }
    float   batteryPct()     const { return battery_pct_; }
    float   cpuTemp()        const { return cpu_temp_; }
    float   gpuTemp()        const { return gpu_temp_; }
    int     wifiSignal()     const { return wifi_signal_; }
    bool    cameraOnline()   const { return camera_online_; }
    bool    lidarOnline()    const { return lidar_online_; }
    bool    voiceOnline()    const { return voice_online_; }
    float   linearVel()      const { return linear_vel_; }
    int     robotMode()      const { return robot_mode_; }
    bool    sleeping()       const { return sleeping_; }

signals:
    void stateChanged();
    void currentUserChanged();
    void lastUtteranceChanged();
    void lastResponseChanged();
    void batteryVoltageChanged();
    void batteryPctChanged();
    void cpuTempChanged();
    void gpuTempChanged();
    void wifiSignalChanged();
    void cameraOnlineChanged();
    void lidarOnlineChanged();
    void voiceOnlineChanged();
    void linearVelChanged();
    void robotModeChanged();
    void sleepingChanged();

public slots:
    void setState(int s) {
        if (state_ != s) { state_ = s; emit stateChanged(); }
    }
    void setCurrentUser(const QString& u) {
        if (currentUser_ != u) { currentUser_ = u; emit currentUserChanged(); }
    }
    void setLastUtterance(const QString& t) {
        lastUtterance_ = t; emit lastUtteranceChanged();
    }
    void setLastResponse(const QString& t) {
        lastResponse_ = t;
        emit lastResponseChanged();
    }
    void returnToListening() { setState(STATE_LISTENING); }

    void setBatteryVoltage(float v) {
        if (battery_voltage_ != v) { battery_voltage_ = v; emit batteryVoltageChanged(); }
    }
    void setBatteryPct(float p) {
        if (battery_pct_ != p) { battery_pct_ = p; emit batteryPctChanged(); }
    }
    void setCpuTemp(float t) {
        if (cpu_temp_ != t) { cpu_temp_ = t; emit cpuTempChanged(); }
    }
    void setGpuTemp(float t) {
        if (gpu_temp_ != t) { gpu_temp_ = t; emit gpuTempChanged(); }
    }
    void setWifiSignal(int s) {
        if (wifi_signal_ != s) { wifi_signal_ = s; emit wifiSignalChanged(); }
    }
    void setCameraOnline(bool b) {
        if (camera_online_ != b) { camera_online_ = b; emit cameraOnlineChanged(); }
    }
    void setLidarOnline(bool b) {
        if (lidar_online_ != b) { lidar_online_ = b; emit lidarOnlineChanged(); }
    }
    void setVoiceOnline(bool b) {
        if (voice_online_ != b) { voice_online_ = b; emit voiceOnlineChanged(); }
    }
    void setLinearVel(float v) {
        if (linear_vel_ != v) { linear_vel_ = v; emit linearVelChanged(); }
    }
    void setRobotMode(int m) {
        if (robot_mode_ != m) { robot_mode_ = m; emit robotModeChanged(); }
    }
    void setSleeping(bool b) {
        if (sleeping_ != b) { sleeping_ = b; emit sleepingChanged(); }
    }

private:
    int     state_{STATE_LISTENING};
    QString currentUser_{"Unknown"};
    QString lastUtterance_;
    QString lastResponse_;

    float   battery_voltage_{0.0f};
    float   battery_pct_{0.0f};
    float   cpu_temp_{0.0f};
    float   gpu_temp_{0.0f};
    int     wifi_signal_{-1};
    bool    camera_online_{false};
    bool    lidar_online_{false};
    bool    voice_online_{false};
    float   linear_vel_{0.0f};
    int     robot_mode_{MODE_INTERACTION};
    bool    sleeping_{false};
};

// ── JupiterDisplayNode ────────────────────────────────────────────────────────

class JupiterDisplayNode : public rclcpp::Node {
public:
    explicit JupiterDisplayNode(DisplayBridge* bridge)
    : Node("jupiter_display"), bridge_(bridge)
    {
        cpu_thermal_path_ = find_thermal_path("cpu");
        gpu_thermal_path_ = find_thermal_path("gpu");

        // ── Voice / face recognition topics ──────────────────────────────────
        user_sub_ = create_subscription<std_msgs::msg::String>(
            "/current_user", 10,
            [this](std_msgs::msg::String::SharedPtr msg) {
                QMetaObject::invokeMethod(bridge_, "setCurrentUser",
                    Qt::QueuedConnection,
                    Q_ARG(QString, QString::fromStdString(msg->data)));
            });

        utterance_sub_ = create_subscription<std_msgs::msg::String>(
            "/voice/raw_text", 10,
            [this](std_msgs::msg::String::SharedPtr msg) {
                QMetaObject::invokeMethod(bridge_, "setLastUtterance",
                    Qt::QueuedConnection,
                    Q_ARG(QString, QString::fromStdString(msg->data)));
                QMetaObject::invokeMethod(bridge_, "setState",
                    Qt::QueuedConnection,
                    Q_ARG(int, STATE_PROCESSING));
            });

        response_sub_ = create_subscription<std_msgs::msg::String>(
            "/voice/response_text", 10,
            [this](std_msgs::msg::String::SharedPtr msg) {
                QMetaObject::invokeMethod(bridge_, "setLastResponse",
                    Qt::QueuedConnection,
                    Q_ARG(QString, QString::fromStdString(msg->data)));
                QMetaObject::invokeMethod(bridge_, "setState",
                    Qt::QueuedConnection,
                    Q_ARG(int, STATE_SPEAKING));
            });

        battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
            "/battery/state", 10,
            [this](sensor_msgs::msg::BatteryState::SharedPtr msg) {
                QMetaObject::invokeMethod(bridge_, "setBatteryVoltage",
                    Qt::QueuedConnection,
                    Q_ARG(float, msg->voltage));
                QMetaObject::invokeMethod(bridge_, "setBatteryPct",
                    Qt::QueuedConnection,
                    Q_ARG(float, msg->percentage * 100.0f));
            });

        tts_done_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/voice/tts_done", 1,
            [this](std_msgs::msg::Empty::SharedPtr) {
                QMetaObject::invokeMethod(bridge_, "setState",
                    Qt::QueuedConnection,
                    Q_ARG(int, STATE_LISTENING));
            });

        sleep_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/jupiter/sleeping", 10,
            [this](std_msgs::msg::Bool::SharedPtr msg) {
                QMetaObject::invokeMethod(bridge_, "setSleeping",
                    Qt::QueuedConnection,
                    Q_ARG(bool, msg->data));
            });

        // ── System health subscriptions ───────────────────────────────────────
        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/color/camera_info", 1,
            [this](sensor_msgs::msg::CameraInfo::SharedPtr) {
                last_camera_time_ = Clock::now();
            });

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 1,
            [this](sensor_msgs::msg::LaserScan::SharedPtr) {
                last_lidar_time_ = Clock::now();
            });

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 1,
            [this](geometry_msgs::msg::Twist::SharedPtr msg) {
                last_cmdvel_time_ = Clock::now();
                const float vel = std::hypot(msg->linear.x, msg->linear.y);
                QMetaObject::invokeMethod(bridge_, "setLinearVel",
                    Qt::QueuedConnection,
                    Q_ARG(float, vel));
            });

        // Poll thermal, WiFi, and health state every 2 seconds
        sys_timer_ = create_wall_timer(std::chrono::seconds(2),
            [this]() { update_system_status(); });

        RCLCPP_INFO(get_logger(), "Jupiter Display online");
    }

private:
    void update_system_status() {
        QMetaObject::invokeMethod(bridge_, "setCpuTemp", Qt::QueuedConnection,
            Q_ARG(float, read_thermal(cpu_thermal_path_)));
        QMetaObject::invokeMethod(bridge_, "setGpuTemp", Qt::QueuedConnection,
            Q_ARG(float, read_thermal(gpu_thermal_path_)));
        QMetaObject::invokeMethod(bridge_, "setWifiSignal", Qt::QueuedConnection,
            Q_ARG(int, read_wifi_signal()));

        const auto now = Clock::now();
        const auto thresh = std::chrono::seconds(3);

        QMetaObject::invokeMethod(bridge_, "setCameraOnline", Qt::QueuedConnection,
            Q_ARG(bool, (now - last_camera_time_) < thresh));
        QMetaObject::invokeMethod(bridge_, "setLidarOnline", Qt::QueuedConnection,
            Q_ARG(bool, (now - last_lidar_time_) < thresh));

        // Voice health: check if the jupiter_voice node is alive in the graph
        const auto names = get_node_names();
        const bool voice_ok = std::find(names.begin(), names.end(),
                                        "/jupiter_voice") != names.end();
        QMetaObject::invokeMethod(bridge_, "setVoiceOnline", Qt::QueuedConnection,
            Q_ARG(bool, voice_ok));

        // Navigation mode: cmd_vel received within the last 2 seconds
        const bool navigating = (now - last_cmdvel_time_) < std::chrono::seconds(2);
        QMetaObject::invokeMethod(bridge_, "setRobotMode", Qt::QueuedConnection,
            Q_ARG(int, navigating ? MODE_NAVIGATING : MODE_INTERACTION));
    }

    DisplayBridge* bridge_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr             user_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr             utterance_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr             response_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr              tts_done_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr    battery_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr      camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr       scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr         cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr               sleep_sub_;
    rclcpp::TimerBase::SharedPtr                                       sys_timer_;

    std::string cpu_thermal_path_;
    std::string gpu_thermal_path_;

    using Clock = std::chrono::steady_clock;
    Clock::time_point last_camera_time_{Clock::time_point::min()};
    Clock::time_point last_lidar_time_{Clock::time_point::min()};
    Clock::time_point last_cmdvel_time_{Clock::time_point::min()};
};

// ── main ──────────────────────────────────────────────────────────────────────

#include "display_node.moc"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    QGuiApplication app(argc, argv);

    DisplayBridge bridge;

    auto node = std::make_shared<JupiterDisplayNode>(&bridge);
    std::thread ros_thread([node]() { rclcpp::spin(node); });

    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty("bridge", &bridge);

    const std::string share_dir =
        ament_index_cpp::get_package_share_directory("jupiter_display");
    const QString qml_path =
        QString::fromStdString(share_dir + "/qml/JupiterFace.qml");

    engine.load(QUrl::fromLocalFile(qml_path));
    if (engine.rootObjects().isEmpty()) {
        rclcpp::shutdown();
        return 1;
    }

    auto* window = qobject_cast<QWindow*>(engine.rootObjects().first());
    if (window) {
        QScreen* target = nullptr;
        for (QScreen* screen : QGuiApplication::screens()) {
            if (screen->geometry().width() == 1024 &&
                screen->geometry().height() == 600) {
                target = screen;
                break;
            }
        }
        if (target) {
            window->setScreen(target);
            window->setGeometry(target->geometry());
            window->showFullScreen();
            RCLCPP_INFO(node->get_logger(), "Display on 7-inch panel (%s)",
                        target->name().toStdString().c_str());
        } else {
            RCLCPP_WARN(node->get_logger(), "7-inch panel not found — using primary screen");
        }
    }

    QTimer shutdown_timer;
    QObject::connect(&shutdown_timer, &QTimer::timeout, [&app, window]() {
        if (!rclcpp::ok()) {
            if (window) window->hide();
            app.quit();
        }
    });
    shutdown_timer.start(50);

    const int result = app.exec();
    rclcpp::shutdown();
    if (ros_thread.joinable()) ros_thread.join();
    return result;
}
