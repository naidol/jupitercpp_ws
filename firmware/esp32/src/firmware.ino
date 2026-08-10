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

// Dock contact state from the proximity sensors: bit0 = left, bit1 = right.
// Both bits set = seated square against the dock.
rcl_publisher_t dock_contact_publisher;
std_msgs__msg__UInt8 dock_contact_msg;

// ---- Dock charge-enable emitter (TSAL6400, 38kHz LEDC burst packets) ----
// The envelope runs in a FreeRTOS task pinned to core 0 (loop()/micro-ROS own core 1),
// because the 600us burst timing can't survive the executor's ~10ms spin blocking.
// ledcWrite from task context is safe; ir_emit_active is the single control flag.
volatile bool ir_emit_active = false;

void irEmitterTask(void *arg)
{
    (void)arg;
    for (;;) {
        if (ir_emit_active) {
            for (uint8_t i = 0; i < IR_BURSTS_PER_PKT && ir_emit_active; i++) {
                ledcWrite(IR_EMIT_LEDC_CH, 128);            // 50% duty @ 38kHz = carrier ON
                delayMicroseconds(IR_BURST_ON_US);
                ledcWrite(IR_EMIT_LEDC_CH, 0);              // carrier OFF
                delayMicroseconds(IR_BURST_OFF_US);
            }
            vTaskDelay(pdMS_TO_TICKS(IR_PACKET_GAP_MS));    // AGC-reset gap between packets
        } else {
            ledcWrite(IR_EMIT_LEDC_CH, 0);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

// ---- Dock contact + charge state (updated each timer cycle) ----
bool prox_left_contact  = false;
bool prox_right_contact = false;
bool dock_seated        = false;   // debounced both-sensors latch (gates the charge beacon)
static uint8_t  seat_debounce    = 0;
static float    last_battery_v   = 0.0f;
static uint32_t first_contact_ms = 0;   // millis() at FIRST sensor contact; 0 = nothing touching

// ---- cmd_vel watchdog ----
volatile unsigned long last_cmd_vel_ms = 0;

// ---- Position-control (segment) mode: /wheel_move ----
// Docking drives the base by DISTANCE (encoder counts) instead of speed. See the block in
// jupiter_config.h and docs/DOCK_POSITION_CONTROL_SPEC.md. Velocity mode is untouched: this is
// a second, mutually-exclusive mode, entered only by a /wheel_move message.
rcl_subscription_t wheel_move_subscriber;
std_msgs__msg__Int32MultiArray wheel_move_msg;

rcl_publisher_t wheel_move_state_publisher;
std_msgs__msg__UInt8 wheel_move_state_msg;

rcl_publisher_t wheel_move_remaining_publisher;
std_msgs__msg__Int32MultiArray wheel_move_remaining_msg;

enum : uint8_t {
    MOVE_IDLE          = 0,
    MOVE_RUNNING       = 1,
    MOVE_DONE          = 2,
    MOVE_ABORT_STALL   = 3,
    MOVE_ABORT_TIMEOUT = 4,
    MOVE_ABORT_REJECT  = 5,   // segment longer than MOVE_MAX_SEGMENT_CNT, or malformed
    MOVE_ABORT_DIVERGE = 6    // error growing, not shrinking -> wrong direction (encoder sign?)
};

static volatile bool move_active   = false;   // true = POSITION mode, false = VELOCITY mode
static uint8_t  move_state         = MOVE_IDLE;
static int32_t  move_target_l = 0, move_target_r = 0;   // counts to travel (relative)
static int32_t  move_origin_l = 0, move_origin_r = 0;   // encoder counts when the move started
static float    move_max_rpm       = MOVE_DEFAULT_MAX_RPM;
static float    move_rpm_l = 0.0f, move_rpm_r = 0.0f;   // slew-limited RPM commands
static uint32_t move_start_ms = 0, move_timeout_ms = 0;
static uint32_t move_progress_ms = 0, move_in_tol_since_ms = 0;
static int32_t  move_prog_ref_l = 0, move_prog_ref_r = 0;   // last position that counted as progress

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
// Each motor now takes TWO LEDC channels: pwm-pin channel (0-3) + dir-pin channel (8-11),
// so reverse can PWM the dir pin for fast-decay (brake-mode) drive. IR emitter stays on ch 4.
Motor motor1(MOTOR1_PWM, MOTOR1_DIR, 0, 8,  PWM_FREQUENCY, PWM_BITS);
Motor motor2(MOTOR2_PWM, MOTOR2_DIR, 1, 9,  PWM_FREQUENCY, PWM_BITS);
Motor motor3(MOTOR3_PWM, MOTOR3_DIR, 2, 10, PWM_FREQUENCY, PWM_BITS);
Motor motor4(MOTOR4_PWM, MOTOR4_DIR, 3, 11, PWM_FREQUENCY, PWM_BITS);

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
    last_battery_v = v;   // used by the charge-enable gate (stop emitting at full)
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

// ---- Position-control (segment) mode helpers ----

static inline int32_t absdiff32(int32_t a, int32_t b) { int32_t d = a - b; return d < 0 ? -d : d; }
static inline int32_t iabs32(int32_t v)               { return v < 0 ? -v : v; }

// End the current move and hand the base back to VELOCITY mode cleanly.
// Re-arms the cmd_vel watchdog timestamp so a stale one can't instantly trip on return.
static void moveFinish(uint8_t new_state)
{
    move_active = false;
    move_state  = new_state;
    move_rpm_l  = 0.0f;
    move_rpm_r  = 0.0f;
    target_linear_velocity   = 0;
    target_linear_y_velocity = 0;
    target_angular_velocity  = 0;
    last_cmd_vel_ms = millis();
}

// One step of the position loop. Writes the per-wheel RPM setpoints that the EXISTING velocity
// PID then tracks — this wraps that loop, it does not replace it. Returns false once the move
// has terminated (for any reason), in which case the caller falls back to velocity mode.
//
// Both wheels are scaled by their SHARE of the remaining distance, so they finish together: a
// rotation stays a rotation instead of one wheel arriving first and the robot arcing. Speed is
// proportional to the LARGEST remaining error and capped, giving a clean deceleration into the
// target over ~MOVE_DEFAULT_MAX_RPM / MOVE_K_POS counts.
static bool positionStep(float dt, float *req_l, float *req_r)
{
    const uint32_t now_ms = millis();
    const int32_t  cnt_l  = (int32_t)motor1_encoder.getCount();
    const int32_t  cnt_r  = (int32_t)motor2_encoder.getCount();

    const int32_t rem_l  = move_target_l - (cnt_l - move_origin_l);
    const int32_t rem_r  = move_target_r - (cnt_r - move_origin_r);
    const int32_t arem_l = iabs32(rem_l);
    const int32_t arem_r = iabs32(rem_r);
    const int32_t rem_max = (arem_l > arem_r) ? arem_l : arem_r;

    // --- SEAT REFLEX: contact while reversing means we have ARRIVED at the dock, not failed.
    if ((prox_left_contact && prox_right_contact) && (move_target_l < 0 || move_target_r < 0)) {
        moveFinish(MOVE_DONE);
        return false;
    }

    // --- ARRIVAL: LATCHED. Once both wheels are inside tolerance we stop commanding and simply
    //     hold zero until DONE. The latch matters: with a minimum speed floor, un-latching on a
    //     few counts of drift would re-command MOVE_MIN_RPM and hunt around the target forever.
    //     Same hard-stop-inside-the-deadband rule the in-place SQUARE state uses.
    if (move_in_tol_since_ms != 0 ||
        (arem_l <= MOVE_DONE_TOL_COUNTS && arem_r <= MOVE_DONE_TOL_COUNTS)) {
        if (move_in_tol_since_ms == 0) move_in_tol_since_ms = now_ms;
        *req_l = 0.0f;
        *req_r = 0.0f;
        move_rpm_l = 0.0f;
        move_rpm_r = 0.0f;
        if (now_ms - move_in_tol_since_ms >= MOVE_DONE_HOLD_MS) {
            moveFinish(MOVE_DONE);
            return false;
        }
        return true;   // holding still inside the deadband — skip the stall guard below
    }

    // --- STALL GUARD (mandatory: position mode suspends the cmd_vel watchdog, and a position
    //     loop will push against a blocked wheel indefinitely without this).
    if (absdiff32(cnt_l, move_prog_ref_l) >= MOVE_STALL_MIN_COUNTS ||
        absdiff32(cnt_r, move_prog_ref_r) >= MOVE_STALL_MIN_COUNTS) {
        move_prog_ref_l = cnt_l;
        move_prog_ref_r = cnt_r;
        move_progress_ms = now_ms;
    } else if (now_ms - move_progress_ms > MOVE_STALL_MS) {
        moveFinish(MOVE_ABORT_STALL);
        return false;
    }

    // --- TIMEOUT GUARD
    if (now_ms - move_start_ms > move_timeout_ms) {
        moveFinish(MOVE_ABORT_TIMEOUT);
        return false;
    }

    // --- DIVERGENCE GUARD: the remaining error must SHRINK. If it grows past where it started,
    //     the wheel is travelling the wrong way — an inverted encoder sign, a miswired motor, or
    //     a reversed command. Without this the loop reads a growing error, holds full speed and
    //     runs until the timeout (metres, not centimetres). Fails fast instead.
    if (arem_l > iabs32(move_target_l) + MOVE_DIVERGE_COUNTS ||
        arem_r > iabs32(move_target_r) + MOVE_DIVERGE_COUNTS) {
        moveFinish(MOVE_ABORT_DIVERGE);
        return false;
    }

    // --- control law: proportional, but never below the speed the drivetrain can actually
    //     execute. A pure proportional law crawls to a halt short of target (measured: 1.6 RPM
    //     asked at 27 counts remaining -> wheels dead -> stall abort at 73/100). Floor first,
    //     then the caller's cap, so an explicit max_rpm is still respected.
    float speed = MOVE_K_POS * (float)rem_max;          // decelerate as the target approaches
    if (speed < MOVE_MIN_RPM)   speed = MOVE_MIN_RPM;   // ...but keep the wheels actually turning
    if (speed > move_max_rpm)   speed = move_max_rpm;

    float want_l = 0.0f, want_r = 0.0f;
    if (rem_max > 0) {
        want_l = speed * ((float)rem_l / (float)rem_max);   // share of remaining -> finish together
        want_r = speed * ((float)rem_r / (float)rem_max);
    }

    // slew-limit to a trapezoidal profile so segments don't jerk the chassis
    const float dv = MOVE_ACCEL_RPM_S * dt;
    move_rpm_l += constrain(want_l - move_rpm_l, -dv, dv);
    move_rpm_r += constrain(want_r - move_rpm_r, -dv, dv);

    // NOTE: MOTOR1/2_TRIM deliberately NOT applied here. Trim compensates motor mismatch in the
    // open-loop velocity path; in position mode the encoder counts are the authority and the
    // loop corrects mismatch itself — trimming would corrupt the commanded distance.
    *req_l = move_rpm_l;
    *req_r = move_rpm_r;
    return true;
}

// Duty scale that holds torque constant as the pack discharges (see jupiter_config.h).
// Returns exactly 1.0 if the battery reading is missing or implausible, so a bad ADC can
// never scale the motors up.
static inline float voltageScale()
{
    if (last_battery_v < BATTERY_V_MIN || last_battery_v > (BATTERY_V_MAX + 0.5f)) return 1.0f;
    float scale = MOTOR_V_NOMINAL / last_battery_v;
    if (scale < MOTOR_V_COMP_MIN) scale = MOTOR_V_COMP_MIN;
    if (scale > MOTOR_V_COMP_MAX) scale = MOTOR_V_COMP_MAX;
    return scale;
}

void moveBase(float dt)
{
    float current_rpm1, current_rpm2, current_rpm3, current_rpm4;
    float target_rpm1,  target_rpm2,  target_rpm3,  target_rpm4;

    // ---- MODE ARBITRATION ----------------------------------------------------------------
    // /cmd_vel is the ONLY channel Nav2 has to the motors, so position control is a separate,
    // mutually-exclusive MODE rather than an addition to the velocity path. In position mode the
    // cmd_vel watchdog below is deliberately SUSPENDED (nothing publishes cmd_vel during a move,
    // so it would brake the wheels 400 ms in); positionStep()'s stall + timeout guards are its
    // safety equivalent. Velocity mode is byte-for-byte the previous behaviour.
    bool position_mode = false;
    float pos_req_l = 0.0f, pos_req_r = 0.0f;
    if (move_active) {
        position_mode = positionStep(dt, &pos_req_l, &pos_req_r);
    }

    // cmd_vel WATCHDOG: publisher died / network hiccup -> stop, don't latch the last command.
    // VELOCITY MODE ONLY — see the arbitration note above.
    if (!position_mode && millis() - last_cmd_vel_ms > CMD_VEL_TIMEOUT_MS) {
        target_linear_velocity   = 0;
        target_linear_y_velocity = 0;
        target_angular_velocity  = 0;
    }

    // Prox REFLEX (endstop-style): any dock contact blocks further REVERSE drive instantly;
    // when fully seated also block rotation (no grinding against the dock). FORWARD stays
    // allowed always — that's how the robot undocks.
    // SEAT REFLEX. Stopping on the FIRST sensor was wrong: on a slightly skewed entry the near
    // sensor trips, reverse halts, and the far sensor can then NEVER close its gap — the robot
    // parks forever at one-sensor, dock_seated never latches, charging never starts.
    // Instead: keep easing in after first contact so the guide rails finish squaring the robot,
    // and stop when BOTH sensors confirm (immediate, undebounced) — or when the grace window
    // expires, so a sensor that never engages can't grind us into the dock indefinitely.
    const bool both_contact = prox_left_contact && prox_right_contact;
    const bool seat_grace_expired =
        (first_contact_ms != 0) && (millis() - first_contact_ms > CONTACT_SEAT_GRACE_MS);
    if ((both_contact || seat_grace_expired) && target_linear_velocity < 0) {
        target_linear_velocity = 0;
    }
    if (dock_seated) {
        target_angular_velocity = 0;
    }

    Kinematics::MotorRPM req_rpm;
    if (position_mode) {
        // Setpoints come from the position loop; the velocity PID below tracks them unchanged.
        req_rpm.motor1 = pos_req_l;
        req_rpm.motor2 = pos_req_r;
        req_rpm.motor3 = 0.0f;
        req_rpm.motor4 = 0.0f;
    } else {
        req_rpm = kinematics.calculateRPM(
            target_linear_velocity, target_linear_y_velocity, target_angular_velocity);

        // Apply per-motor trim to the TARGET RPM (not PWM output) so PID cannot compensate it away.
        req_rpm.motor1 *= MOTOR1_TRIM;
        req_rpm.motor2 *= MOTOR2_TRIM;
    }

    current_rpm1 = motor1_encoder.getRPM();
    current_rpm2 = motor2_encoder.getRPM();
    // Motors 3 & 4 removed (rear caster config) — zero out to prevent free-spinning
    // shafts from corrupting PID feedback and odometry.
    current_rpm3 = 0.0f;
    current_rpm4 = 0.0f;

    // Idle brake — VELOCITY MODE ONLY. Must not fire in position mode: the cmd_vel setpoints are
    // not the control variable there and are typically zero, so this would brake the wheels for
    // the whole segment (and again during every ramp through zero).
    if (!position_mode &&
        target_linear_velocity == 0 && target_linear_y_velocity == 0 && target_angular_velocity == 0) {
        motor1_pid.compute(req_rpm.motor1, current_rpm1, dt);
        motor2_pid.compute(req_rpm.motor2, current_rpm2, dt);
        target_rpm1 = 0.0f;
        target_rpm2 = 0.0f;
        target_rpm3 = 0.0f;
        target_rpm4 = 0.0f;
    } else if (USE_PID == true) {
        target_rpm1 = motor1_pid.compute(req_rpm.motor1, current_rpm1, dt);
        target_rpm2 = motor2_pid.compute(req_rpm.motor2, current_rpm2, dt);
        // Breakaway-ONLY static feed-forward: kick a wheel toward motion ONLY while it is
        // stalled AND meaningfully commanded — assists the initial breakaway (both wheels
        // start together, no arc) then gets out of the way once rolling so the PID does clean
        // fine steering. A continuous kick slammed tiny reverse heading corrections and made
        // the robot over-rotate off the dock.
        // ⚠️ The 4 RPM gate below causes uncommanded yaw at crawl speed — see the KNOWN ISSUE
        // note in jupiter_config.h. Mitigation for now: do not operate below ~0.10 m/s.
        if (fabsf(req_rpm.motor1) > MOTOR_FF_CMD_MIN && fabsf(current_rpm1) < MOTOR_FF_RELEASE_RPM)
            target_rpm1 += copysignf(MOTOR_FF_STATIC, req_rpm.motor1);
        if (fabsf(req_rpm.motor2) > MOTOR_FF_CMD_MIN && fabsf(current_rpm2) < MOTOR_FF_RELEASE_RPM)
            target_rpm2 += copysignf(MOTOR_FF_STATIC, req_rpm.motor2);
        target_rpm3 = 0.0f;
        target_rpm4 = 0.0f;
    } else {
        target_rpm1 = req_rpm.motor1;
        target_rpm2 = req_rpm.motor2;
        target_rpm3 = 0.0f;
        target_rpm4 = 0.0f;
    }

    // Hold torque constant as the pack discharges: the tune is done at the docking-condition
    // voltage (the robot docks BECAUSE it is low), and this keeps the same tune honest at full
    // charge instead of ~17 % hotter. Applied to PID+FF together — both are duties, and the
    // torque both produce scales with pack voltage. Zero stays zero (brake).
    const float vscale = voltageScale();
    motor1.setSpeed(target_rpm1 * vscale);
    motor2.setSpeed(target_rpm2 * vscale);
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
    last_cmd_vel_ms = millis();   // feed the watchdog

    // A NON-ZERO cmd_vel always wins: it cancels any in-flight position move and drops straight
    // back to velocity mode. This is what keeps teleop usable as an e-stop while a segment runs.
    // A zero cmd_vel does NOT cancel — velocity_smoother and idle publishers emit zeros
    // continuously, and those must not kill a docking segment.
    if (move_active &&
        (msg->linear.x != 0.0 || msg->linear.y != 0.0 || msg->angular.z != 0.0)) {
        move_active = false;
        move_state  = MOVE_IDLE;
        move_rpm_l  = 0.0f;
        move_rpm_r  = 0.0f;
    }
}

// /wheel_move — [counts_left, counts_right, (optional) max_rpm]. RELATIVE counts from wherever
// the wheels are now, so there is no shared origin to drift. A new message supersedes any
// in-flight move (last command wins).
void wheelMoveCallback(const void * msgin)
{
    const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)msgin;
    if (msg->data.size < 2) { moveFinish(MOVE_ABORT_REJECT); return; }

    const int32_t counts_l = msg->data.data[0];
    const int32_t counts_r = msg->data.data[1];

    // Reject absurd segments — guards against a bad computation driving the robot across the room.
    if (iabs32(counts_l) > MOVE_MAX_SEGMENT_CNT || iabs32(counts_r) > MOVE_MAX_SEGMENT_CNT) {
        moveFinish(MOVE_ABORT_REJECT);
        return;
    }

    move_max_rpm = (msg->data.size >= 3 && msg->data.data[2] > 0)
                   ? (float)msg->data.data[2] : MOVE_DEFAULT_MAX_RPM;
    const float rpm_ceiling = (float)MOTOR_MAX_RPM * (float)MAX_RPM_RATIO;
    if (move_max_rpm > rpm_ceiling) move_max_rpm = rpm_ceiling;

    move_origin_l = (int32_t)motor1_encoder.getCount();
    move_origin_r = (int32_t)motor2_encoder.getCount();
    move_target_l = counts_l;
    move_target_r = counts_r;
    move_prog_ref_l = move_origin_l;
    move_prog_ref_r = move_origin_r;

    // Generous completion budget: expected time at the speed cap, x factor, with a 1 s floor.
    const int32_t longest = (iabs32(counts_l) > iabs32(counts_r)) ? iabs32(counts_l) : iabs32(counts_r);
    const float   revs    = (float)longest / (float)COUNTS_PER_REV1;
    const float   secs    = (move_max_rpm > 0.1f) ? (revs / (move_max_rpm / 60.0f)) : 1.0f;
    move_timeout_ms = (uint32_t)(secs * 1000.0f * MOVE_TIMEOUT_FACTOR) + 1000;

    const uint32_t now_ms = millis();
    move_start_ms        = now_ms;
    move_progress_ms     = now_ms;
    move_in_tol_since_ms = 0;
    move_rpm_l = 0.0f;
    move_rpm_r = 0.0f;
    move_state  = MOVE_RUNNING;
    move_active = true;
}

void timerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer == NULL) return;

    unsigned long now = millis();
    float dt = (now - prev_clk_time) / 1000.0f;
    prev_clk_time = now;

    // Poll dock proximity sensors BEFORE moveBase so the reflex uses this cycle's truth.
    prox_left_contact  = (digitalRead(PROX_LEFT_PIN)  == LOW);
    prox_right_contact = (digitalRead(PROX_RIGHT_PIN) == LOW);

    // Debounced "seated" latch: both sensors held for PROX_DEBOUNCE_CYCLES to set,
    // both clear for the same count to release (no SSR chatter from contact bounce).
    if (prox_left_contact && prox_right_contact) {
        if (seat_debounce < PROX_DEBOUNCE_CYCLES) seat_debounce++;
        if (seat_debounce >= PROX_DEBOUNCE_CYCLES) dock_seated = true;
    } else if (!prox_left_contact && !prox_right_contact) {
        if (seat_debounce > 0) seat_debounce--;
        if (seat_debounce == 0) dock_seated = false;
    }

    // Seat-grace timer: starts at the FIRST sensor contact, clears when nothing is touching.
    if (prox_left_contact || prox_right_contact) {
        if (first_contact_ms == 0) first_contact_ms = millis();
    } else {
        first_contact_ms = 0;
    }

    // Charge-enable beacon: seated AND battery not full. Everything else is implied:
    // undock/drag-away/fault -> not seated -> silence -> dock SSR opens in ~300ms.
    ir_emit_active = dock_seated && (last_battery_v > 1.0f) && (last_battery_v < BATTERY_FULL_STOP);

    moveBase(dt);
    publishData();

    encoder_msg.data.data[0] = (int32_t)motor1_encoder.getCount();
    encoder_msg.data.data[1] = (int32_t)motor2_encoder.getCount();
    encoder_msg.data.data[2] = (int32_t)motor3_encoder.getCount();
    encoder_msg.data.data[3] = (int32_t)motor4_encoder.getCount();
    RCSOFTCHECK(rcl_publish(&encoder_publisher, &encoder_msg, NULL));

    // Battery at 1 Hz
    EXECUTE_EVERY_N_MS(1000, publish_battery(););

    // Dock contact state: bit0 = left prox, bit1 = right prox (3 = seated square).
    dock_contact_msg.data = (prox_left_contact ? 1 : 0) | (prox_right_contact ? 2 : 0);
    RCSOFTCHECK(rcl_publish(&dock_contact_publisher, &dock_contact_msg, NULL));

    // Position-mode telemetry: the aligner waits on state to sequence its segments.
    wheel_move_state_msg.data = move_state;
    RCSOFTCHECK(rcl_publish(&wheel_move_state_publisher, &wheel_move_state_msg, NULL));

    wheel_move_remaining_msg.data.data[0] =
        move_active ? (move_target_l - ((int32_t)motor1_encoder.getCount() - move_origin_l)) : 0;
    wheel_move_remaining_msg.data.data[1] =
        move_active ? (move_target_r - ((int32_t)motor2_encoder.getCount() - move_origin_r)) : 0;
    RCSOFTCHECK(rcl_publish(&wheel_move_remaining_publisher, &wheel_move_remaining_msg, NULL));
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
        &dock_contact_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        "/dock/contact"));

    CREATE_CHECK(rclc_subscription_init_default(
        &cmd_vel_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    CREATE_CHECK(rclc_subscription_init_default(
        &save_imu_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
        "/save_imu"));

    // Position-control (segment) mode — see jupiter_config.h and DOCK_POSITION_CONTROL_SPEC.md
    CREATE_CHECK(rclc_subscription_init_default(
        &wheel_move_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "/wheel_move"));

    CREATE_CHECK(rclc_publisher_init_default(
        &wheel_move_state_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        "/wheel_move_state"));

    CREATE_CHECK(rclc_publisher_init_default(
        &wheel_move_remaining_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "/wheel_move_remaining"));

    CREATE_CHECK(rclc_timer_init_default(
        &timer, &support,
        RCL_MS_TO_NS(20),
        timerCallback));

    // Handles: 1 timer + 3 subscriptions. MUST match the number added below — rclc silently
    // drops anything beyond the declared count, so a stale value here makes /wheel_move look
    // connected while its callback never fires.
    CREATE_CHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
    CREATE_CHECK(rclc_executor_add_timer(&executor, &timer));
    CREATE_CHECK(rclc_executor_add_subscription(
        &executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA));
    CREATE_CHECK(rclc_executor_add_subscription(
        &executor, &save_imu_subscriber, &save_imu_msg, &saveImuCallback, ON_NEW_DATA));
    CREATE_CHECK(rclc_executor_add_subscription(
        &executor, &wheel_move_subscriber, &wheel_move_msg, &wheelMoveCallback, ON_NEW_DATA));

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
    (void)rcl_publisher_fini(&dock_contact_publisher, &node);
    (void)rcl_publisher_fini(&wheel_move_state_publisher,     &node);
    (void)rcl_publisher_fini(&wheel_move_remaining_publisher, &node);
    (void)rcl_subscription_fini(&cmd_vel_subscriber,  &node);
    (void)rcl_subscription_fini(&save_imu_subscriber, &node);
    (void)rcl_subscription_fini(&wheel_move_subscriber, &node);
    (void)rcl_timer_fini(&timer);
    (void)rclc_executor_fini(&executor);
    (void)rcl_node_fini(&node);
    (void)rclc_support_fini(&support);
}

// ---- Arduino entry points ----

void setup()
{
    Serial.begin(460800);  // micro-ROS serial transport — must match agent -b. 921600 was UNRELIABLE (periodic session drops from serial corruption); 460800 keeps ~15Hz headroom with more timing margin.

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
    Wire.setClock(400000);   // BNO055 Fast Mode (4x the default 100kHz) — cuts IMU read time
    setup_oled_display();
    setup_imu(&imu_msg);
    setup_battery_adc();

    pinMode(ESP32_LED, OUTPUT);
    digitalWrite(ESP32_LED, LOW);
    flashLED(5);

    // Dock proximity sensors — NPN open-collector, only sink: internal pull-ups give a
    // defined HIGH when clear; sensor pulls LOW on metal (dock contact). Polled at 50Hz
    // in timerCallback (slow digital signal — no interrupts needed).
    pinMode(PROX_LEFT_PIN,  INPUT_PULLUP);
    pinMode(PROX_RIGHT_PIN, INPUT_PULLUP);

    // Dock charge-enable emitter: 38kHz carrier on LEDC, envelope task on core 0
    // (micro-ROS owns core 1; the 600us burst timing must not fight the executor).
    ledcSetup(IR_EMIT_LEDC_CH, 38000, 8);
    ledcAttachPin(IR_EMIT_PIN, IR_EMIT_LEDC_CH);
    ledcWrite(IR_EMIT_LEDC_CH, 0);
    xTaskCreatePinnedToCore(irEmitterTask, "ir_emit", 2048, NULL, 1, NULL, 0);

    // Pre-allocate message data arrays once (reused across reconnects)
    encoder_msg.data.size     = 4;
    encoder_msg.data.capacity = 4;
    encoder_msg.data.data     = (int32_t *)malloc(4 * sizeof(int32_t));

    speed_msg.data.size     = 5;
    speed_msg.data.capacity = 5;
    speed_msg.data.data     = (float_t *)malloc(5 * sizeof(float_t));

    // /wheel_move INCOMING buffer. micro-ROS will not allocate for an unbounded sequence on
    // receive — without capacity here the deserialiser drops or overruns the message.
    // [counts_left, counts_right, max_rpm, flags] = 4 elements, plus a dim slot for the layout.
    wheel_move_msg.data.size         = 0;
    wheel_move_msg.data.capacity     = 4;
    wheel_move_msg.data.data         = (int32_t *)malloc(4 * sizeof(int32_t));
    wheel_move_msg.layout.dim.size     = 0;
    wheel_move_msg.layout.dim.capacity = 1;
    wheel_move_msg.layout.dim.data     =
        (std_msgs__msg__MultiArrayDimension *)malloc(sizeof(std_msgs__msg__MultiArrayDimension));
    wheel_move_msg.layout.dim.data[0].label.size     = 0;
    wheel_move_msg.layout.dim.data[0].label.capacity = 24;
    wheel_move_msg.layout.dim.data[0].label.data     = (char *)malloc(24);

    wheel_move_remaining_msg.data.size     = 2;
    wheel_move_remaining_msg.data.capacity = 2;
    wheel_move_remaining_msg.data.data     = (int32_t *)malloc(2 * sizeof(int32_t));

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
            // Cancel any in-flight position move. Without this move_active survives the outage
            // and the segment RESUMES on reconnect — the robot would lurch into a stale command
            // issued before the link dropped, with no one waiting on it.
            move_active = false;
            move_state  = MOVE_IDLE;
            move_rpm_l  = 0.0f;
            move_rpm_r  = 0.0f;
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
