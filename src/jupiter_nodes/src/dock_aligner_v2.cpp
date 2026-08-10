// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace {
constexpr int D_VALID = 0;
constexpr int D_ALONG = 1;
constexpr int D_LATERAL = 2;
constexpr int D_RANGE = 3;
constexpr int D_BEARING = 4;
constexpr int D_SKEW = 5;

double wrap_pi(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }
double clampd(double value, double low, double high) { return std::max(low, std::min(high, value)); }
}  // namespace

class DockAlignerV2 : public rclcpp::Node {
public:
  DockAlignerV2() : Node("dock_aligner_v2") {
    reflector_topic_ = declare_parameter<std::string>("reflector_topic", "/dock/reflector");
    confidence_topic_ = declare_parameter<std::string>("confidence_topic", "/dock/reflector_confidence");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
    contact_topic_ = declare_parameter<std::string>("contact_topic", "/dock/contact");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    state_topic_ = declare_parameter<std::string>("state_topic", "/dock/v2/aligner_state");
    start_service_ = declare_parameter<std::string>("start_service", "/dock/v2/align_start");
    cancel_service_ = declare_parameter<std::string>("cancel_service", "/dock/v2/align_cancel");
    calibrate_positive_service_ = declare_parameter<std::string>(
      "calibrate_positive_service", "/dock/v2/calibrate_positive");
    calibrate_negative_service_ = declare_parameter<std::string>(
      "calibrate_negative_service", "/dock/v2/calibrate_negative");

    control_hz_ = declare_parameter("control_hz", 20.0);
    v_reverse_ = declare_parameter("v_reverse", 0.12);
    v_reverse_throat_ = declare_parameter("v_reverse_throat", 0.12);
    throat_boost_range_ = declare_parameter("throat_boost_range", 0.30);
    v_seat_push_ = declare_parameter("v_seat_push", 0.12);
    seat_push_timeout_s_ = declare_parameter("seat_push_timeout_s", 4.0);
    max_angular_ = declare_parameter("max_angular", 0.22);

    min_confidence_ = declare_parameter("min_confidence", 0.70);
    reflector_stale_s_ = declare_parameter("reflector_stale_s", 0.25);
    invalid_abort_s_ = declare_parameter("invalid_abort_s", 0.8);
    acquire_stable_s_ = declare_parameter("acquire_stable_s", 0.5);
    acquire_timeout_s_ = declare_parameter("acquire_timeout_s", 12.0);
    overall_timeout_s_ = declare_parameter("overall_timeout_s", 90.0);

    heading_sign_ = declare_parameter("heading_sign", -1.0);
    kp_square_ = declare_parameter("kp_square", 0.9);
    square_tol_ = declare_parameter("square_tol_deg", 1.2) * M_PI / 180.0;
    square_settle_s_ = declare_parameter("square_settle_s", 0.35);
    square_timeout_s_ = declare_parameter("square_timeout_s", 20.0);
    square_omega_min_ = declare_parameter("square_omega_min", 0.14);
    square_omega_max_ = declare_parameter("square_omega_max", 0.30);
    square_diverge_step_deg_ = declare_parameter("square_diverge_step_deg", 1.5);
    square_diverge_count_max_ = declare_parameter("square_diverge_count_max", 6);

    seated_range_m_ = declare_parameter("seated_range_m", 0.1937);
    seated_lateral_m_ = declare_parameter("seated_lateral_m", 0.0039);
    seated_nyaw_ = declare_parameter("seated_nyaw_deg", 0.96) * M_PI / 180.0;
    seat_range_tolerance_m_ = declare_parameter("seat_range_tolerance_m", 0.020);

    no_progress_window_s_ = declare_parameter("no_progress_window_s", 5.0);
    no_progress_min_delta_m_ = declare_parameter("no_progress_min_delta_m", 0.015);
    stall_boost_step_ = declare_parameter("stall_boost_step", 0.02);
    stall_boost_max_v_ = declare_parameter("stall_boost_max_v", 0.14);
    stall_check_s_ = declare_parameter("stall_check_s", 0.7);
    stall_min_delta_m_ = declare_parameter("stall_min_delta_m", 0.008);

    kp_pursuit_ = declare_parameter("kp_pursuit", 2.0);
    kd_yaw_rate_ = declare_parameter("kd_yaw_rate", 2.0);
    close_aim_hold_m_ = declare_parameter("close_aim_hold_m", 0.60);
    // Bench 2026-08-08: wheels deliver ~40-70%% of commanded RPM; differential is
    // crushed below ~0.10 m/s. 0.12 m/s with az<=0.22 is the measured authority floor.
    min_wheel_speed_ = declare_parameter("min_wheel_speed", 0.045);
    half_track_m_ = declare_parameter("half_track_m", 0.1775);
    midgate_range_ = declare_parameter("midgate_range", 0.45);
    midgate_lateral_max_ = declare_parameter("midgate_lateral_max", 0.045);
    midgate_heading_max_ = declare_parameter("midgate_heading_max_deg", 8.0) * M_PI / 180.0;
    acquire_min_range_ = declare_parameter("acquire_min_range", 0.85);
    acquire_lateral_max_ = declare_parameter("acquire_lateral_max", 0.120);
    acquire_heading_max_ = declare_parameter("acquire_heading_max_deg", 8.0) * M_PI / 180.0;
    reverse_lateral_abort_ = declare_parameter("reverse_lateral_abort", 0.20);
    reverse_heading_abort_ = declare_parameter("reverse_heading_abort_deg", 20.0) * M_PI / 180.0;
    docking_enabled_ = declare_parameter("docking_enabled", true);
    calibration_angular_ = declare_parameter("calibration_angular", 0.14);
    calibration_pulse_s_ = declare_parameter("calibration_pulse_s", 0.50);
    calibration_settle_s_ = declare_parameter("calibration_settle_s", 1.0);

    one_contact_abort_range_ = declare_parameter(
      "one_contact_abort_range", seated_range_m_ + seat_range_tolerance_m_);
    hard_stop_range_ = declare_parameter(
      "hard_stop_range", seated_range_m_ - seat_range_tolerance_m_);

    square_only_ = declare_parameter("square_only", false);
    reverse_test_mode_ = declare_parameter("reverse_test_mode", false);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic_, rclcpp::QoS(1).transient_local());

    refl_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      reflector_topic_, 10, std::bind(&DockAlignerV2::on_reflector, this, std::placeholders::_1));
    conf_sub_ = create_subscription<std_msgs::msg::Float32>(
      confidence_topic_, 10, std::bind(&DockAlignerV2::on_confidence, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(), std::bind(&DockAlignerV2::on_imu, this, std::placeholders::_1));
    contact_sub_ = create_subscription<std_msgs::msg::UInt8>(
      contact_topic_, 10, std::bind(&DockAlignerV2::on_contact, this, std::placeholders::_1));

    start_srv_ = create_service<std_srvs::srv::Trigger>(
      start_service_, std::bind(&DockAlignerV2::on_start, this, std::placeholders::_1, std::placeholders::_2));
    cancel_srv_ = create_service<std_srvs::srv::Trigger>(
      cancel_service_, std::bind(&DockAlignerV2::on_cancel, this, std::placeholders::_1, std::placeholders::_2));
    calibrate_positive_srv_ = create_service<std_srvs::srv::Trigger>(
      calibrate_positive_service_,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        on_calibrate(request, response, 1.0);
      });
    calibrate_negative_srv_ = create_service<std_srvs::srv::Trigger>(
      calibrate_negative_service_,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        on_calibrate(request, response, -1.0);
      });

    const auto period = std::chrono::duration<double>(1.0 / control_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&DockAlignerV2::on_tick, this));

    set_state(IDLE);
    RCLCPP_INFO(
      get_logger(),
      "dock_aligner_v2 ready. start=%s, state=%s, reflector seat target: range=%.4fm lateral=%+.4fm nyaw=%+.2fdeg",
      start_service_.c_str(), state_topic_.c_str(), seated_range_m_, seated_lateral_m_,
      seated_nyaw_ * 180.0 / M_PI);
  }

private:
  enum State { IDLE, ACQUIRE, SQUARE, REVERSE_IN, CALIBRATE, SEATED, ABORT };

  const char *state_name(State state) const {
    switch (state) {
      case IDLE: return "IDLE";
      case ACQUIRE: return "ACQUIRE";
      case SQUARE: return "SQUARE";
      case REVERSE_IN: return "REVERSE_IN";
      case CALIBRATE: return "CALIBRATE";
      case SEATED: return "SEATED";
      default: return "ABORT";
    }
  }

  void set_state(State next) {
    state_ = next;
    std_msgs::msg::String message;
    message.data = state_name(next);
    state_pub_->publish(message);
    RCLCPP_INFO(get_logger(), "state -> %s", state_name(next));
  }

  void on_reflector(const std_msgs::msg::Float32MultiArray::SharedPtr message) {
    if (message->data.size() < 6) {
      return;
    }
    refl_valid_ = (message->data[D_VALID] > 0.5f);
    refl_along_ = message->data[D_ALONG];
    refl_lateral_ = message->data[D_LATERAL];
    refl_range_ = message->data[D_RANGE];
    const double bearing = message->data[D_BEARING];
    const double skew = message->data[D_SKEW];
    refl_nyaw_ = wrap_pi(skew + bearing + M_PI);
    last_refl_time_ = now();
  }

  void on_confidence(const std_msgs::msg::Float32::SharedPtr message) { refl_confidence_ = message->data; }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr message) {
    const auto &q = message->orientation;
    yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    // Light low-pass on gyro z for the pursuit damping term.
    gyro_z_ = 0.8 * gyro_z_ + 0.2 * message->angular_velocity.z;
    have_imu_ = true;
  }

  void on_contact(const std_msgs::msg::UInt8::SharedPtr message) { contact_mask_ = message->data; }

  bool pose_ok() const {
    if (!refl_valid_ || refl_confidence_ < min_confidence_) {
      return false;
    }
    const double age = (now() - last_refl_time_).seconds();
    return age <= reflector_stale_s_;
  }

  bool both_contact() const { return (contact_mask_ & 0x03) == 0x03; }

  bool one_contact() const {
    const uint8_t pair = contact_mask_ & 0x03;
    return pair == 0x01 || pair == 0x02;
  }

  double square_angular_cmd() const {
    const double error = heading_error();
    double cmd = heading_sign_ * kp_square_ * error;
    const double magnitude = std::fabs(cmd);
    if (magnitude < square_omega_min_) {
      cmd = (cmd >= 0.0 ? 1.0 : -1.0) * square_omega_min_;
    }
    return clampd(cmd, -square_omega_max_, square_omega_max_);
  }

  double heading_error() const { return wrap_pi(refl_nyaw_ - seated_nyaw_); }

  double lateral_error() const { return refl_lateral_ - seated_lateral_m_; }

  // Bearing (relative to the robot's REAR axis) of a pursuit point on the dock approach axis.
  // Far out the point sits along the dock normal so the robot arrives square; inside
  // close_aim_hold it collapses onto the dock centre, dropping dependence on the
  // arc-distorted close-range nyaw.
  double pursuit_mu() const {
    const double standoff =
      std::min(0.5 * refl_range_, std::max(0.0, refl_range_ - close_aim_hold_m_));
    const double aim_x = refl_along_ + std::cos(refl_nyaw_) * standoff;
    const double aim_y = refl_lateral_ + std::sin(refl_nyaw_) * standoff - seated_lateral_m_;
    return std::atan2(-aim_y, -aim_x);
  }

  double reverse_angular_cmd(double speed) const {
    // Cap angular so both wheel speeds stay above breakaway (stalled wheel = inverted turn).
    const double wheel_cap = std::max(0.0, (speed - min_wheel_speed_) / half_track_m_);
    const double cap = std::min(max_angular_, wheel_cap);
    // Yaw plant is heavily lagged at creep speed (fitted tau ~7.5s); damp on measured rate.
    return clampd(kp_pursuit_ * pursuit_mu() - kd_yaw_rate_ * gyro_z_, -cap, cap);
  }

  void stop_robot() {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }

  void on_start(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (!docking_enabled_) {
      response->success = false;
      response->message = "v2 docking disabled pending measured steering calibration";
      return;
    }
    if (state_ == ACQUIRE || state_ == SQUARE || state_ == REVERSE_IN || state_ == CALIBRATE) {
      response->success = false;
      response->message = "v2 already running";
      return;
    }

    run_start_ = now();
    acquire_start_ = run_start_;
    valid_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    square_enter_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    square_in_band_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_pose_ok_ = run_start_;
    have_yaw_lock_ = false;
    square_runtime_sign_ = heading_sign_;
    square_diverge_count_ = 0;
    last_square_nyaw_ = std::numeric_limits<double>::quiet_NaN();
    progress_window_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    progress_window_range_start_ = 0.0;
    progress_window_best_range_ = 0.0;
    seat_push_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    if (reverse_test_mode_) {
      // Focused test mode: skip ACQUIRE/SQUARE and immediately exercise REVERSE_IN behavior.
      yaw_lock_ = yaw_;
      have_yaw_lock_ = true;
      progress_window_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      seat_push_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      stall_check_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      stall_boost_ = 0.0;
      midgate_checked_ = false;
      set_state(REVERSE_IN);
      response->success = true;
      response->message = "dock_aligner_v2 reverse-test mode started";
      return;
    }

    set_state(ACQUIRE);
    response->success = true;
    response->message = "dock_aligner_v2 started";
  }

  void on_cancel(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    stop_robot();
    set_state(ABORT);
    response->success = true;
    response->message = "dock_aligner_v2 canceled";
  }

  void on_calibrate(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response,
    double direction) {
    if (state_ == ACQUIRE || state_ == SQUARE || state_ == REVERSE_IN || state_ == CALIBRATE) {
      response->success = false;
      response->message = "v2 already running";
      return;
    }
    if ((contact_mask_ & 0x03) != 0) {
      response->success = false;
      response->message = "calibration refused: dock contact is nonzero";
      return;
    }
    if (!pose_ok()) {
      response->success = false;
      response->message = "calibration refused: reflector pose invalid";
      return;
    }

    calibration_direction_ = direction;
    calibration_start_ = now();
    calibration_stop_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    calibration_start_range_ = refl_range_;
    calibration_start_lateral_ = refl_lateral_;
    calibration_start_nyaw_ = refl_nyaw_;
    run_start_ = calibration_start_;
    last_pose_ok_ = calibration_start_;
    set_state(CALIBRATE);
    response->success = true;
    response->message = direction > 0.0 ? "positive calibration pulse started" :
      "negative calibration pulse started";
  }

  void on_tick() {
    if (state_ == IDLE || state_ == SEATED || state_ == ABORT) {
      return;
    }

    if ((now() - run_start_).seconds() > overall_timeout_s_) {
      RCLCPP_WARN(get_logger(), "overall timeout -> ABORT");
      stop_robot();
      set_state(ABORT);
      return;
    }

    const bool valid_pose = pose_ok();
    if (valid_pose) {
      last_pose_ok_ = now();
    } else if ((now() - last_pose_ok_).seconds() > invalid_abort_s_) {
      RCLCPP_WARN(get_logger(), "pose invalid/stale for %.2fs -> ABORT", invalid_abort_s_);
      stop_robot();
      set_state(ABORT);
      return;
    }

    switch (state_) {
      case ACQUIRE: {
        if (!valid_pose) {
          if ((now() - acquire_start_).seconds() > acquire_timeout_s_) {
            RCLCPP_WARN(get_logger(), "acquire timeout -> ABORT");
            stop_robot();
            set_state(ABORT);
          }
          return;
        }

        if (valid_since_.nanoseconds() == 0) {
          valid_since_ = now();
        }

        if ((now() - valid_since_).seconds() >= acquire_stable_s_) {
          if (refl_range_ < acquire_min_range_ ||
            std::fabs(lateral_error()) > acquire_lateral_max_ ||
            std::fabs(heading_error()) > acquire_heading_max_)
          {
            RCLCPP_WARN(
              get_logger(),
              "capture envelope rejected pose: range %.3f (min %.2f), lateral error %+.3fm (max %.3f), heading error %+.1fdeg (max %.1f) -> ABORT",
              refl_range_, acquire_min_range_, lateral_error(), acquire_lateral_max_,
              heading_error() * 180.0 / M_PI, acquire_heading_max_ * 180.0 / M_PI);
            stop_robot();
            set_state(ABORT);
            return;
          }
          if (square_only_) {
            square_enter_ = now();
            square_in_band_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
            square_runtime_sign_ = heading_sign_;
            square_diverge_count_ = 0;
            last_square_nyaw_ = std::numeric_limits<double>::quiet_NaN();
            set_state(SQUARE);
          } else {
            progress_window_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
            seat_push_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
            stall_check_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
            stall_boost_ = 0.0;
            midgate_checked_ = false;
            RCLCPP_INFO(
              get_logger(),
              "ACQUIRE stable: mu %+.2fdeg, lat err %+.3fm, heading err %+.2fdeg -> pursuit reverse",
              pursuit_mu() * 180.0 / M_PI, lateral_error(), heading_error() * 180.0 / M_PI);
            set_state(REVERSE_IN);
          }
        }
        return;
      }

      case SQUARE: {
        if (!valid_pose) {
          return;
        }
        if ((now() - square_enter_).seconds() > square_timeout_s_) {
          RCLCPP_WARN(get_logger(), "square timeout -> ABORT");
          stop_robot();
          set_state(ABORT);
          return;
        }

        const double calibrated_heading_error = heading_error();
        const double abs_heading_error = std::fabs(calibrated_heading_error);
        if (abs_heading_error <= square_tol_) {
          if (square_in_band_since_.nanoseconds() == 0) {
            square_in_band_since_ = now();
          }

          stop_robot();
          if ((now() - square_in_band_since_).seconds() >= square_settle_s_) {
            yaw_lock_ = yaw_;
            have_yaw_lock_ = true;
            RCLCPP_INFO(
              get_logger(), "SQUARE lock: nyaw=%+.2fdeg target=%+.2fdeg error=%+.2fdeg",
              refl_nyaw_ * 180.0 / M_PI, seated_nyaw_ * 180.0 / M_PI,
              calibrated_heading_error * 180.0 / M_PI);

            if (square_only_) {
              set_state(ABORT);
            } else {
              progress_window_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
              set_state(REVERSE_IN);
            }
          }
          return;
        }

        square_in_band_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

        const double magnitude = clampd(kp_square_ * abs_heading_error, square_omega_min_, square_omega_max_);
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = square_runtime_sign_ * std::copysign(magnitude, calibrated_heading_error);
        cmd_pub_->publish(cmd);

        if (std::isfinite(last_square_nyaw_)) {
          const double step = square_diverge_step_deg_ * M_PI / 180.0;
          const bool same_side = (calibrated_heading_error * last_square_nyaw_) > 0.0;
          const bool worsening = abs_heading_error > std::fabs(last_square_nyaw_) + step;
          if (same_side && worsening) {
            ++square_diverge_count_;
          } else {
            square_diverge_count_ = std::max(0, square_diverge_count_ - 1);
          }
          if (square_diverge_count_ >= square_diverge_count_max_) {
            square_runtime_sign_ = -square_runtime_sign_;
            square_diverge_count_ = 0;
            RCLCPP_WARN(
              get_logger(),
              "SQUARE divergence detected (heading error %+.1fdeg). Flipping runtime sign to %+.0f",
              calibrated_heading_error * 180.0 / M_PI, square_runtime_sign_);
          }
        }
        last_square_nyaw_ = calibrated_heading_error;

        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 700,
          "V2 SQUARE: nyaw %+.1fdeg | target %+.1fdeg | error %+.1fdeg | az %+.2f",
          refl_nyaw_ * 180.0 / M_PI, seated_nyaw_ * 180.0 / M_PI,
          calibrated_heading_error * 180.0 / M_PI, cmd.angular.z);
        return;
      }

      case REVERSE_IN: {
        if (!valid_pose) {
          return;
        }

        // nyaw arc-distorts below ~0.35m and rails own the throat; envelope applies outside only.
        if (refl_range_ > throat_boost_range_ &&
          (std::fabs(lateral_error()) > reverse_lateral_abort_ ||
          std::fabs(heading_error()) > reverse_heading_abort_))
        {
          RCLCPP_WARN(
            get_logger(),
            "reverse pose left safety envelope: lateral error %+.3fm, heading error %+.1fdeg -> ABORT",
            lateral_error(), heading_error() * 180.0 / M_PI);
          stop_robot();
          set_state(ABORT);
          return;
        }

        if (both_contact()) {
          stop_robot();
          set_state(SEATED);
          return;
        }

        if (refl_range_ <= hard_stop_range_) {
          RCLCPP_WARN(get_logger(), "hard stop range %.3f reached without both contacts -> ABORT", refl_range_);
          stop_robot();
          set_state(ABORT);
          return;
        }

        // Seat zone: funnel rails own alignment; push straight for a bounded time.
        if (refl_range_ <= one_contact_abort_range_) {
          if (seat_push_start_.nanoseconds() == 0) {
            seat_push_start_ = now();
            RCLCPP_INFO(
              get_logger(), "seat zone entered (range %.3f, contact %u) -> compliant push",
              refl_range_, contact_mask_ & 0x03);
          }
          if ((now() - seat_push_start_).seconds() > seat_push_timeout_s_) {
            RCLCPP_WARN(
              get_logger(),
              "seat push timeout (%.1fs, range %.3f, contact %u) -> ABORT",
              seat_push_timeout_s_, refl_range_, contact_mask_ & 0x03);
            stop_robot();
            set_state(ABORT);
            return;
          }
        }

        if (progress_window_start_.nanoseconds() == 0) {
          progress_window_start_ = now();
          progress_window_range_start_ = refl_range_;
          progress_window_best_range_ = refl_range_;
        } else {
          progress_window_best_range_ = std::min(progress_window_best_range_, refl_range_);
          if ((now() - progress_window_start_).seconds() > no_progress_window_s_) {
            const double progress = progress_window_range_start_ - progress_window_best_range_;
            if (progress < no_progress_min_delta_m_) {
              RCLCPP_WARN(
                get_logger(),
                "no reverse progress (start %.3f, best %.3f, delta %.3f in %.2fs) -> ABORT",
                progress_window_range_start_, progress_window_best_range_, progress,
                no_progress_window_s_);
              stop_robot();
              set_state(ABORT);
              return;
            }
            progress_window_start_ = now();
            progress_window_range_start_ = refl_range_;
            progress_window_best_range_ = refl_range_;
          }
        }

        const double mu = pursuit_mu();
        // Mid-course gate: last checkpoint with steering authority before the throat.
        if (refl_range_ <= midgate_range_ && !midgate_checked_) {
          midgate_checked_ = true;
          if (std::fabs(lateral_error()) > midgate_lateral_max_ ||
            std::fabs(heading_error()) > midgate_heading_max_)
          {
            RCLCPP_WARN(
              get_logger(),
              "midgate reject at range %.3f: lat err %+.3fm (max %.3f), heading err %+.1fdeg (max %.1f) -> ABORT",
              refl_range_, lateral_error(), midgate_lateral_max_,
              heading_error() * 180.0 / M_PI, midgate_heading_max_ * 180.0 / M_PI);
            stop_robot();
            set_state(ABORT);
            return;
          }
          RCLCPP_INFO(
            get_logger(), "midgate PASS at range %.3f: lat err %+.3fm, heading err %+.1fdeg",
            refl_range_, lateral_error(), heading_error() * 180.0 / M_PI);
        }

        const bool in_throat = (refl_range_ <= throat_boost_range_);
        const bool in_seat_zone = (refl_range_ <= one_contact_abort_range_);
        double speed = in_seat_zone ? v_seat_push_ : (in_throat ? v_reverse_throat_ : v_reverse_);

        // Caster breakaway / rail friction: step speed up while range is not closing.
        if (stall_check_start_.nanoseconds() == 0) {
          stall_check_start_ = now();
          stall_check_range_ = refl_range_;
        } else if ((now() - stall_check_start_).seconds() > stall_check_s_) {
          if (stall_check_range_ - refl_range_ < stall_min_delta_m_) {
            stall_boost_ = std::min(stall_boost_ + stall_boost_step_, stall_boost_max_v_);
            RCLCPP_INFO(
              get_logger(), "stall boost +%.2f (total %.2f) at range %.3f",
              stall_boost_step_, stall_boost_, refl_range_);
          } else if (stall_boost_ > 0.0) {
            stall_boost_ = std::max(0.0, stall_boost_ - stall_boost_step_);
          }
          stall_check_start_ = now();
          stall_check_range_ = refl_range_;
        }
        speed = std::min(speed + stall_boost_, stall_boost_max_v_);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = -speed;
        // Rails out-torque the wheels in the throat; steering there only scrubs.
        cmd.angular.z = in_throat ? 0.0 : reverse_angular_cmd(speed);

        cmd_pub_->publish(cmd);

        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 700,
          "V2 PURSUIT: range %.3f (seat_err %+.3f) | mu %+.1fdeg | nyaw %+.1fdeg (err %+.1f) | lat %+.3f (err %+.3f) | v %.2f az %+.2f | contact %u",
          refl_range_, refl_range_ - seated_range_m_, mu * 180.0 / M_PI,
          refl_nyaw_ * 180.0 / M_PI, heading_error() * 180.0 / M_PI, refl_lateral_,
          lateral_error(), cmd.linear.x, cmd.angular.z, contact_mask_ & 0x03);
        return;
      }

      case CALIBRATE: {
        if ((contact_mask_ & 0x03) != 0 || !valid_pose) {
          RCLCPP_WARN(get_logger(), "calibration safety interlock -> ABORT");
          stop_robot();
          set_state(ABORT);
          return;
        }

        if ((now() - calibration_start_).seconds() < calibration_pulse_s_) {
          geometry_msgs::msg::Twist cmd;
          cmd.angular.z = calibration_direction_ * calibration_angular_;
          cmd_pub_->publish(cmd);
          return;
        }

        stop_robot();
        if (calibration_stop_.nanoseconds() == 0) {
          calibration_stop_ = now();
          return;
        }
        if ((now() - calibration_stop_).seconds() < calibration_settle_s_) {
          return;
        }

        RCLCPP_INFO(
          get_logger(),
          "CALIBRATION RESULT: cmd_az=%+.3f pulse=%.2fs | range %.4f->%.4f delta=%+.4f | lateral %+.4f->%+.4f delta=%+.4f | nyaw %+.2f->%+.2fdeg delta=%+.2fdeg",
          calibration_direction_ * calibration_angular_, calibration_pulse_s_,
          calibration_start_range_, refl_range_, refl_range_ - calibration_start_range_,
          calibration_start_lateral_, refl_lateral_, refl_lateral_ - calibration_start_lateral_,
          calibration_start_nyaw_ * 180.0 / M_PI, refl_nyaw_ * 180.0 / M_PI,
          wrap_pi(refl_nyaw_ - calibration_start_nyaw_) * 180.0 / M_PI);
        set_state(ABORT);
        return;
      }

      default:
        return;
    }
  }

  // Topics/services.
  std::string reflector_topic_;
  std::string confidence_topic_;
  std::string imu_topic_;
  std::string contact_topic_;
  std::string cmd_vel_topic_;
  std::string state_topic_;
  std::string start_service_;
  std::string cancel_service_;
  std::string calibrate_positive_service_;
  std::string calibrate_negative_service_;

  // Parameters.
  double control_hz_{20.0};
  double v_reverse_{0.06};
  double v_reverse_throat_{0.08};
  double throat_boost_range_{0.26};
  double v_seat_push_{0.10};
  double seat_push_timeout_s_{4.0};
  rclcpp::Time seat_push_start_{0, 0, RCL_ROS_TIME};
  double max_angular_{0.25};

  double min_confidence_{0.70};
  double reflector_stale_s_{0.25};
  double invalid_abort_s_{0.8};
  double acquire_stable_s_{0.5};
  double acquire_timeout_s_{12.0};
  double overall_timeout_s_{90.0};

  double heading_sign_{-1.0};
  double kp_square_{0.9};
  double square_tol_{1.2 * M_PI / 180.0};
  double square_settle_s_{0.35};
  double square_timeout_s_{20.0};
  double square_omega_min_{0.14};
  double square_omega_max_{0.30};
  double square_diverge_step_deg_{1.5};
  int square_diverge_count_max_{6};

  double seated_range_m_{0.1937};
  double seated_lateral_m_{0.0039};
  double seated_nyaw_{0.96 * M_PI / 180.0};
  double seat_range_tolerance_m_{0.020};

  double no_progress_window_s_{5.0};
  double no_progress_min_delta_m_{0.015};
  double stall_boost_step_{0.02};
  double stall_boost_max_v_{0.14};
  double stall_check_s_{0.7};
  double stall_min_delta_m_{0.008};
  double stall_boost_{0.0};
  rclcpp::Time stall_check_start_{0, 0, RCL_ROS_TIME};
  double stall_check_range_{0.0};

  double kp_pursuit_{0.7};
  double kd_yaw_rate_{2.0};
  double close_aim_hold_m_{0.60};
  double min_wheel_speed_{0.045};
  double half_track_m_{0.1775};
  double midgate_range_{0.45};
  double midgate_lateral_max_{0.045};
  double midgate_heading_max_{8.0 * M_PI / 180.0};
  bool midgate_checked_{false};
  double acquire_min_range_{0.85};
  double acquire_lateral_max_{0.120};
  double acquire_heading_max_{8.0 * M_PI / 180.0};
  double reverse_lateral_abort_{0.20};
  double reverse_heading_abort_{20.0 * M_PI / 180.0};
  bool docking_enabled_{true};
  double calibration_angular_{0.14};
  double calibration_pulse_s_{0.50};
  double calibration_settle_s_{1.0};

  double one_contact_abort_range_{0.2137};
  double hard_stop_range_{0.1737};

  bool square_only_{false};
  bool reverse_test_mode_{false};

  // Runtime.
  State state_{IDLE};
  bool refl_valid_{false};
  double refl_along_{0.0};
  double refl_lateral_{0.0};
  double refl_range_{99.0};
  double refl_nyaw_{0.0};
  double refl_confidence_{0.0};

  bool have_imu_{false};
  double yaw_{0.0};
  double gyro_z_{0.0};
  bool have_yaw_lock_{false};
  double yaw_lock_{0.0};
  double square_runtime_sign_{1.0};
  int square_diverge_count_{0};
  double last_square_nyaw_{0.0};

  double calibration_direction_{0.0};
  double calibration_start_range_{0.0};
  double calibration_start_lateral_{0.0};
  double calibration_start_nyaw_{0.0};

  uint8_t contact_mask_{0};

  rclcpp::Time run_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time acquire_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time valid_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time square_enter_{0, 0, RCL_ROS_TIME};
  rclcpp::Time calibration_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time calibration_stop_{0, 0, RCL_ROS_TIME};
  rclcpp::Time square_in_band_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time progress_window_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_refl_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pose_ok_{0, 0, RCL_ROS_TIME};

  double progress_window_range_start_{0.0};
  double progress_window_best_range_{0.0};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr refl_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr conf_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr contact_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_positive_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_negative_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DockAlignerV2>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
