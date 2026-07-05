//#######################################################################################################
// Name:             main.cpp
// Purpose:          Jupiter Robot ESP32 firmware
// Description:      Robot motor drivers, IMU, LED are controlled using PID and communicates to Host PC
//                   using Micro-ROS.  This firmware reads cmd_vel msgs from ROS2 host and publishes
//                   imu/data and odom/unfiltered msgs back to the host so that ROS2 Navigation can compute
//                   the robots position and orientation and determine velocity feedback to the ESP32
//                   Also included are other modules that drive the attached OLED display and Onboard LED
//                   to indicate when the Robot is listening to voice commands.
// Related Files:    this firmware is built to compile on VS CODE using the PLATFORMIO plugin
// Author:           logan naidoo, south africa, 2024
//########################################################################################################

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <Wire.h>
#include <esp32-hal-ledc.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/battery_state.h>
#include <utility/imumaths.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/empty.h>
#include <std_msgs/msg/u_int8.h>

#include "jupiter_config.h"
#include "imu_bno055.h"
#include "encoder.h"
#include "kinematics.h"
#include "odometry.h"
#include "pid.h"
#include "motor.h"

// RCCHECK halts on unrecoverable errors (hardware setup). CREATE_CHECK returns false so the
// state machine can handle micro-ROS entity creation failures gracefully.
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define CREATE_CHECK(fn) { rcl_ret_t _rc = (fn); if(_rc != RCL_RET_OK) { return false; } }

// Run a block at most once every MS milliseconds using a static timer.
#define EXECUTE_EVERY_N_MS(MS, X) do { \
    static unsigned long _t = 0; \
    if (millis() - _t >= (unsigned long)(MS)) { X; _t = millis(); } \
} while (0)

// 4-state micro-ROS reconnection state machine
typedef enum {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} agent_state_t;

static agent_state_t agent_state = WAITING_AGENT;

// External declarations from imu_bno055.cpp
extern bool trigger_imu_save;
void perform_imu_save();

// ---- micro-ROS entities ----
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;

rcl_subscription_t save_imu_subscriber;
std_msgs__msg__Empty save_imu_msg;

rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;

rcl_publisher_t encoder_publisher;
std_msgs__msg__Int32MultiArray encoder_msg;

rcl_publisher_t speed_publisher;
std_msgs__msg__Float32MultiArray speed_msg;

rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry odom_msg;

rcl_publisher_t battery_publisher;
sensor_msgs__msg__BatteryState battery_msg;

rcl_publisher_t dock_ir_publisher;
std_msgs__msg__UInt8 dock_ir_msg;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;

// ---- Odometry state ----
unsigned long long time_offset = 0;
float prev_clk_time = 0;
unsigned long prev_odom_update = 0;
Odometry odometry;

// ---- ADC state ----
static esp_adc_cal_characteristics_t adc_chars;

// ---- Battery moving average (10-sample / 10-second window at 1 Hz) ----
static float   bat_ma_buf[BATTERY_MA_SIZE] = {};
static uint8_t bat_ma_idx   = 0;
static uint8_t bat_ma_count = 0;
static float   bat_ma_sum   = 0.0f;

// ---- Hardware instances ----
Motor motor1(MOTOR1_PWM, MOTOR1_DIR, 0, PWM_FREQUENCY, PWM_BITS);
Motor motor2(MOTOR2_PWM, MOTOR2_DIR, 1, PWM_FREQUENCY, PWM_BITS);
Motor motor3(MOTOR3_PWM, MOTOR3_DIR, 2, PWM_FREQUENCY, PWM_BITS);
Motor motor4(MOTOR4_PWM, MOTOR4_DIR, 3, PWM_FREQUENCY, PWM_BITS);

Encoder motor1_encoder(MOTOR1_ENC_A, MOTOR1_ENC_B, COUNTS_PER_REV1);
Encoder motor2_encoder(MOTOR2_ENC_A, MOTOR2_ENC_B, COUNTS_PER_REV2);
Encoder motor3_encoder(MOTOR3_ENC_A, MOTOR3_ENC_B, COUNTS_PER_REV3);
Encoder motor4_encoder(MOTOR4_ENC_A, MOTOR4_ENC_B, COUNTS_PER_REV4);

Kinematics kinematics(WHEEL_RADIUS, WHEEL_SEPARATION, WHEEL_BASE);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

float target_linear_velocity   = 0;
float target_linear_y_velocity = 0.0f;
float target_angular_velocity  = 0;

// ---- Utility helpers ----

void flashLED(int n_times)
{
    for (int i = 0; i < n_times; i++) {
        digitalWrite(ESP32_LED, HIGH);
        delay(150);
        digitalWrite(ESP32_LED, LOW);
        delay(150);
    }
    delay(1000);
}

void rclErrorLoop()
{
    while (true) {
        flashLED(2);
    }
}

void syncTime()
{
    unsigned long now = millis();
    // Non-fatal: skip time sync if agent is busy rather than entering rclErrorLoop.
    // 1000ms timeout handles Jetson startup load (Whisper/camera initialising).
    if (rmw_uros_sync_session(1000) != RMW_RET_OK) return;
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    unsigned long long now = millis() + time_offset;
    tp.tv_sec  = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}

// ---- Battery ADC ----

void setup_battery_adc()
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(BATTERY_ADC_CHANNEL, BATTERY_ADC_ATTEN);
    esp_adc_cal_characterize(ADC_UNIT_1, BATTERY_ADC_ATTEN, ADC_WIDTH_BIT_12, 1100, &adc_chars);
}

float read_battery_voltage()
{
    uint32_t adc_raw = 0;
    for (int i = 0; i < 16; i++) {
        adc_raw += adc1_get_raw(BATTERY_ADC_CHANNEL);
    }
    adc_raw /= 16;
    uint32_t mv = esp_adc_cal_raw_to_voltage(adc_raw, &adc_chars);
    return (mv / 1000.0f) / BATTERY_V_DIV;
}

void publish_battery()
{
    float v_raw = read_battery_voltage();

    // Circular-buffer moving average — subtract oldest sample, add newest
    bat_ma_sum -= bat_ma_buf[bat_ma_idx];
    bat_ma_buf[bat_ma_idx] = v_raw;
    bat_ma_sum += v_raw;
    bat_ma_idx = (bat_ma_idx + 1) % BATTERY_MA_SIZE;
    if (bat_ma_count < BATTERY_MA_SIZE) bat_ma_count++;

    float v = bat_ma_sum / bat_ma_count;
    float pct = (v - BATTERY_V_MIN) / (BATTERY_V_MAX - BATTERY_V_MIN);
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 1.0f) pct = 1.0f;

    struct timespec ts = getTime();
    battery_msg.header.stamp.sec     = ts.tv_sec;
    battery_msg.header.stamp.nanosec = ts.tv_nsec;
    battery_msg.voltage              = v;
    battery_msg.percentage           = pct;
    battery_msg.present              = true;
    battery_msg.power_supply_status     = 2; // DISCHARGING
    battery_msg.power_supply_health     = 1; // GOOD
    battery_msg.power_supply_technology = 2; // LION

    RCSOFTCHECK(rcl_publish(&battery_publisher, &battery_msg, NULL));
}

// ---- Motion ----

void moveBase(float dt)
{
    float current_rpm1, current_rpm2, current_rpm3, current_rpm4;
    float target_rpm1,  target_rpm2,  target_rpm3,  target_rpm4;

    Kinematics::MotorRPM req_rpm = kinematics.calculateRPM(
        target_linear_velocity, target_linear_y_velocity, target_angular_velocity);

    // Apply per-motor trim to the TARGET RPM (not PWM output) so PID cannot compensate it away.
    req_rpm.motor1 *= MOTOR1_TRIM;
    req_rpm.motor2 *= MOTOR2_TRIM;

    current_rpm1 = motor1_encoder.getRPM();
    current_rpm2 = motor2_encoder.getRPM();
    // Motors 3 & 4 removed (rear caster config) — zero out to prevent free-spinning
    // shafts from corrupting PID feedback and odometry.
    current_rpm3 = 0.0f;
    current_rpm4 = 0.0f;

    if (target_linear_velocity == 0 && target_linear_y_velocity == 0 && target_angular_velocity == 0) {
        motor1_pid.compute(req_rpm.motor1, current_rpm1, dt);
        motor2_pid.compute(req_rpm.motor2, current_rpm2, dt);
        target_rpm1 = 0.0f;
        target_rpm2 = 0.0f;
        target_rpm3 = 0.0f;
        target_rpm4 = 0.0f;
    } else if (USE_PID == true) {
        target_rpm1 = motor1_pid.compute(req_rpm.motor1, current_rpm1, dt);
        target_rpm2 = motor2_pid.compute(req_rpm.motor2, current_rpm2, dt);
        target_rpm3 = 0.0f;
        target_rpm4 = 0.0f;
    } else {
        target_rpm1 = req_rpm.motor1;
        target_rpm2 = req_rpm.motor2;
        target_rpm3 = 0.0f;
        target_rpm4 = 0.0f;
    }

    motor1.setSpeed(target_rpm1);
    motor2.setSpeed(target_rpm2);
    motor3.setSpeed(0);
    motor4.setSpeed(0);

    speed_msg.data.data[0] = current_rpm1;
    speed_msg.data.data[1] = current_rpm2;
    speed_msg.data.data[2] = 0.0f;
    speed_msg.data.data[3] = 0.0f;
    speed_msg.data.data[4] = (float_t)dt;
    RCSOFTCHECK(rcl_publish(&speed_publisher, &speed_msg, NULL));

    // Diff-drive odometry: front-left + front-right only (rear motors have no wheels).
    Kinematics::Velocities current_vel = kinematics.getDiffVelocities(
        current_rpm1, current_rpm2);
    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0f;
    odometry.update(vel_dt, current_vel.linear_x, current_vel.linear_y, current_vel.angular_z);
    prev_odom_update = now;
}

void publishData()
{
    odom_msg = odometry.getData();
    get_imu_data(&imu_msg);

    struct timespec time_stamp = getTime();

    odom_msg.header.stamp.sec     = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    imu_msg.header.stamp.sec      = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec  = time_stamp.tv_nsec;

    RCSOFTCHECK(rcl_publish(&imu_publisher,  &imu_msg,  NULL));
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}

// ---- Callbacks ----

void saveImuCallback(const void * msgin)
{
    (void)msgin;
    trigger_imu_save = true;
}

void cmdVelCallback(const void * msgin)
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    target_linear_velocity   = msg->linear.x;
    target_linear_y_velocity = msg->linear.y;
    target_angular_velocity  = msg->angular.z;
}

void timerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer == NULL) return;

    unsigned long now = millis();
    float dt = (now - prev_clk_time) / 1000.0f;
    prev_clk_time = now;

    moveBase(dt);
    publishData();

    encoder_msg.data.data[0] = (int32_t)motor1_encoder.getCount();
    encoder_msg.data.data[1] = (int32_t)motor2_encoder.getCount();
    encoder_msg.data.data[2] = (int32_t)motor3_encoder.getCount();
    encoder_msg.data.data[3] = (int32_t)motor4_encoder.getCount();
    RCSOFTCHECK(rcl_publish(&encoder_publisher, &encoder_msg, NULL));

    // Battery at 1 Hz
    EXECUTE_EVERY_N_MS(1000, publish_battery(););

    // IR dock receivers — bit0=left detected, bit1=right detected
    // KY-022 OUT is active LOW: LOW pin = beam detected
    uint8_t ir_left  = (digitalRead(IR_RECV_LEFT)  == LOW) ? IR_DOCK_LEFT  : 0;
    uint8_t ir_right = (digitalRead(IR_RECV_RIGHT) == LOW) ? IR_DOCK_RIGHT : 0;
    dock_ir_msg.data = ir_left | ir_right;
    RCSOFTCHECK(rcl_publish(&dock_ir_publisher, &dock_ir_msg, NULL));
}

// ---- micro-ROS entity lifecycle ----

bool create_entities()
{
    allocator = rcl_get_default_allocator();
    CREATE_CHECK(rclc_support_init(&support, 0, NULL, &allocator));
    CREATE_CHECK(rclc_node_init_default(&node, "esp32_node", "", &support));

    CREATE_CHECK(rclc_publisher_init_default(
        &imu_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"));

    CREATE_CHECK(rclc_publisher_init_default(
        &odom_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom/unfiltered"));

    CREATE_CHECK(rclc_publisher_init_default(
        &encoder_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "wheel_encoders"));

    CREATE_CHECK(rclc_publisher_init_default(
        &speed_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "wheel_speeds"));

    CREATE_CHECK(rclc_publisher_init_default(
        &battery_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
        "/battery/state"));

    CREATE_CHECK(rclc_publisher_init_default(
        &dock_ir_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        "/dock/ir"));

    CREATE_CHECK(rclc_subscription_init_default(
        &cmd_vel_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    CREATE_CHECK(rclc_subscription_init_default(
        &save_imu_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
        "/save_imu"));

    CREATE_CHECK(rclc_timer_init_default(
        &timer, &support,
        RCL_MS_TO_NS(20),
        timerCallback));

    // Handles: 1 timer + 2 subscriptions
    CREATE_CHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    CREATE_CHECK(rclc_executor_add_timer(&executor, &timer));
    CREATE_CHECK(rclc_executor_add_subscription(
        &executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA));
    CREATE_CHECK(rclc_executor_add_subscription(
        &executor, &save_imu_subscriber, &save_imu_msg, &saveImuCallback, ON_NEW_DATA));

    return true;
}

void destroy_entities()
{
    // Allow destruction to proceed immediately even when the agent is unreachable.
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    (void)rcl_publisher_fini(&imu_publisher,     &node);
    (void)rcl_publisher_fini(&odom_publisher,    &node);
    (void)rcl_publisher_fini(&encoder_publisher, &node);
    (void)rcl_publisher_fini(&speed_publisher,   &node);
    (void)rcl_publisher_fini(&battery_publisher, &node);
    (void)rcl_publisher_fini(&dock_ir_publisher, &node);
    (void)rcl_subscription_fini(&cmd_vel_subscriber,  &node);
    (void)rcl_subscription_fini(&save_imu_subscriber, &node);
    (void)rcl_timer_fini(&timer);
    (void)rclc_executor_fini(&executor);
    (void)rcl_node_fini(&node);
    (void)rclc_support_fini(&support);
}

// ---- Arduino entry points ----

void setup()
{
    Serial.begin(115200);

    // --- Hardware init (blocking delays are safe here, before micro-ROS) ---
    motor1.setSpeed(0);
    motor2.setSpeed(0);
    motor3.setSpeed(0);
    motor4.setSpeed(0);

    motor1_encoder.begin();
    motor2_encoder.begin();
    motor3_encoder.begin();
    motor4_encoder.begin();
    delay(3000);
    motor1_encoder.reset();
    motor2_encoder.reset();
    motor3_encoder.reset();
    motor4_encoder.reset();

    Wire.begin(21, 22);
    setup_oled_display();
    setup_imu(&imu_msg);
    setup_battery_adc();

    pinMode(ESP32_LED, OUTPUT);
    digitalWrite(ESP32_LED, LOW);
    flashLED(5);

    // IR dock receivers — KY-022 OUT is open-drain active LOW; pull-up ensures HIGH when no signal
    pinMode(IR_RECV_LEFT,  INPUT_PULLUP);
    pinMode(IR_RECV_RIGHT, INPUT_PULLUP);

    // Pre-allocate message data arrays once (reused across reconnects)
    encoder_msg.data.size     = 4;
    encoder_msg.data.capacity = 4;
    encoder_msg.data.data     = (int32_t *)malloc(4 * sizeof(int32_t));

    speed_msg.data.size     = 5;
    speed_msg.data.capacity = 5;
    speed_msg.data.data     = (float_t *)malloc(5 * sizeof(float_t));

    // Transport is set once; support/node/entities are rebuilt on each reconnect.
    set_microros_serial_transports(Serial);
}

void loop()
{
    switch (agent_state) {
        case WAITING_AGENT:
            // Ping every 200 ms; advance only when agent responds.
            EXECUTE_EVERY_N_MS(200,
                agent_state = (rmw_uros_ping_agent(200, 3) == RMW_RET_OK)
                              ? AGENT_AVAILABLE : WAITING_AGENT;
            );
            break;

        case AGENT_AVAILABLE:
            if (create_entities()) {
                syncTime();
                agent_state = AGENT_CONNECTED;
                flashLED(3);
            } else {
                destroy_entities();
                delay(2000);  // allow DDS cleanup + host load to settle before retry
                agent_state = WAITING_AGENT;
            }
            break;

        case AGENT_CONNECTED:
            // Check connection every 200 ms; stay connected while agent is alive.
            // 2000ms × 5 attempts = 10s tolerance — survives OrbbecSDK camera
            // init which saturates USB and stalls serial for up to ~8s.
            EXECUTE_EVERY_N_MS(200,
                agent_state = (rmw_uros_ping_agent(2000, 5) == RMW_RET_OK)
                              ? AGENT_CONNECTED : AGENT_DISCONNECTED;
            );
            if (agent_state == AGENT_CONNECTED) {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
                if (trigger_imu_save) {
                    perform_imu_save();
                }
            }
            break;

        case AGENT_DISCONNECTED:
            destroy_entities();
            target_linear_velocity   = 0;
            target_linear_y_velocity = 0;
            target_angular_velocity  = 0;
            motor1.setSpeed(0);
            motor2.setSpeed(0);
            motor3.setSpeed(0);
            motor4.setSpeed(0);
            delay(2000);  // allow DDS cleanup and heap to settle before next create_entities()
            agent_state = WAITING_AGENT;
            break;
    }
}
