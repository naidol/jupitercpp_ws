// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// Jupiter Display Node
// Subscribes to voice and face recognition topics, drives a Qt6 QML animated
// robot face on the onboard 7-inch display.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
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
#include <thread>

// ── Display state ─────────────────────────────────────────────────────────────
// Matches the State enum exposed to QML.
static constexpr int STATE_LISTENING  = 0;
static constexpr int STATE_PROCESSING = 1;
static constexpr int STATE_SPEAKING   = 2;

// ── DisplayBridge ─────────────────────────────────────────────────────────────
// QObject that lives on the Qt main thread. ROS2 callbacks (background thread)
// update it via QMetaObject::invokeMethod with Qt::QueuedConnection, which posts
// to the Qt event loop — fully thread-safe with no manual mutexes.

class DisplayBridge : public QObject {
    Q_OBJECT

    Q_PROPERTY(int     state          READ state          NOTIFY stateChanged)
    Q_PROPERTY(QString currentUser    READ currentUser    NOTIFY currentUserChanged)
    Q_PROPERTY(QString lastUtterance  READ lastUtterance  NOTIFY lastUtteranceChanged)
    Q_PROPERTY(QString lastResponse   READ lastResponse   NOTIFY lastResponseChanged)
    Q_PROPERTY(float   batteryVoltage READ batteryVoltage NOTIFY batteryVoltageChanged)
    Q_PROPERTY(int     speakingMs     READ speakingMs     NOTIFY speakingMsChanged)

public:
    explicit DisplayBridge(QObject* parent = nullptr) : QObject(parent) {}

    int     state()          const { return state_; }
    QString currentUser()    const { return currentUser_; }
    QString lastUtterance()  const { return lastUtterance_; }
    QString lastResponse()   const { return lastResponse_; }
    float   batteryVoltage() const { return battery_voltage_; }
    int     speakingMs()     const { return speaking_ms_; }

signals:
    void stateChanged();
    void currentUserChanged();
    void lastUtteranceChanged();
    void lastResponseChanged();
    void batteryVoltageChanged();
    void speakingMsChanged();

public slots:
    void setState(int s) {
        if (state_ != s) { state_ = s; emit stateChanged(); }
    }

    void setCurrentUser(const QString& u) {
        if (currentUser_ != u) { currentUser_ = u; emit currentUserChanged(); }
    }

    void setLastUtterance(const QString& t) {
        lastUtterance_ = t;
        emit lastUtteranceChanged();
    }

    void setLastResponse(const QString& t) {
        lastResponse_ = t;
        // Estimate TTS duration: ~150 words/min = 400ms/word + 2.5s buffer
        const int words = t.split(' ', Qt::SkipEmptyParts).size();
        speaking_ms_ = words * 400 + 2500;
        emit lastResponseChanged();
        emit speakingMsChanged();
    }

    void setBatteryVoltage(float v) {
        if (battery_voltage_ != v) { battery_voltage_ = v; emit batteryVoltageChanged(); }
    }

    void returnToListening() {
        setState(STATE_LISTENING);
    }

private:
    int     state_{STATE_LISTENING};
    QString currentUser_{"Unknown"};
    QString lastUtterance_;
    QString lastResponse_;
    float   battery_voltage_{0.0f};
    int     speaking_ms_{5000};
};

// ── JupiterDisplayNode ────────────────────────────────────────────────────────

class JupiterDisplayNode : public rclcpp::Node {
public:
    explicit JupiterDisplayNode(DisplayBridge* bridge)
    : Node("jupiter_display"), bridge_(bridge)
    {
        // Who is in front of the camera
        user_sub_ = create_subscription<std_msgs::msg::String>(
            "/current_user", 10,
            [this](std_msgs::msg::String::SharedPtr msg) {
                QMetaObject::invokeMethod(bridge_, "setCurrentUser",
                    Qt::QueuedConnection,
                    Q_ARG(QString, QString::fromStdString(msg->data)));
            });

        // User spoke — switch to PROCESSING while Whisper transcribes
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

        // Jupiter speaking — switch to SPEAKING, QML timer returns to LISTENING
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

        // Battery voltage — optional, displayed in status bar when available
        battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
            "/battery/state", 10,
            [this](sensor_msgs::msg::BatteryState::SharedPtr msg) {
                QMetaObject::invokeMethod(bridge_, "setBatteryVoltage",
                    Qt::QueuedConnection,
                    Q_ARG(float, msg->voltage));
            });

        RCLCPP_INFO(get_logger(), "Jupiter Display online");
    }

private:
    DisplayBridge* bridge_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr       user_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr       utterance_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr       response_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
};

// ── main ──────────────────────────────────────────────────────────────────────

#include "display_node.moc"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    QGuiApplication app(argc, argv);

    DisplayBridge bridge;

    // ROS2 spin in background thread — QMetaObject posts safely to Qt main thread.
    // Capture node by value so the shared_ptr ref-count keeps the node alive for
    // the thread's lifetime.  Do NOT detach — detached threads run past main()'s
    // return, causing rclcpp/DDS objects to be used after their destructors fire.
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

    // Move window to the 7-inch panel (1024x600). Falls back to primary screen
    // if the small display isn't connected.
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

    // Poll rclcpp::ok() so the Qt event loop exits when ROS2 shuts down (Ctrl+C).
    // 50ms interval: responds within one tick of SIGINT instead of up to 200ms.
    QTimer shutdown_timer;
    QObject::connect(&shutdown_timer, &QTimer::timeout, [&app]() {
        if (!rclcpp::ok()) app.quit();
    });
    shutdown_timer.start(50);

    const int result = app.exec();
    rclcpp::shutdown();           // signals spin() to stop (idempotent if already called)
    if (ros_thread.joinable()) ros_thread.join();  // wait for spin() to return cleanly
    return result;
}
