#include "offboard_control/forward_backward.hpp"

using namespace std::chrono_literals;

ForwardBackward::ForwardBackward(const std::string &name) : Node(name)
{
  // QoS sensor data untuk langganan posisi lokal
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

  // === Subscribers ===
  vehicle_local_position_subscriber_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", qos,
      std::bind(&ForwardBackward::local_position_callback, this, std::placeholders::_1));

  // === Publishers ===
  offboard_control_mode_publisher_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
  trajectory_setpoint_publisher_   = create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
  vehicle_command_publisher_       = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

  // === Parameters ===
  target_alt_m_        = declare_parameter<float>("target_alt_m", 2.0f);
  vz_up_               = declare_parameter<float>("vz_up", 1.0f);
  declare_parameter<bool>("start_takeoff", false);

  forward_dist_m_      = declare_parameter<float>("forward_dist_m", 1.5f);
  reach_xy_tol_        = declare_parameter<float>("reach_xy_tol", 0.10f);
  speed_forward_mps_   = declare_parameter<float>("speed_forward_mps", 0.5f);
  slow_k_              = declare_parameter<float>("slow_k", 0.8f);
  vz_hold_max_         = declare_parameter<float>("vz_hold_max", 0.6f);

  // === Dynamic param callback ===
  param_cb_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> &params)
        -> rcl_interfaces::msg::SetParametersResult
    {
      for (const auto &p : params) {
        if (p.get_name() == "start_takeoff") {
          start_takeoff_ = p.as_bool();
          if (start_takeoff_ && initial_position_set_) {
            phase_ = Phase::WAIT_CMD;
            RCLCPP_INFO(this->get_logger(),
              "[CMD] start_takeoff=true diterima (siap OFFBOARD saat stream stabil)");
          }
        } else if (p.get_name() == "target_alt_m") {
          target_alt_m_ = static_cast<float>(p.as_double());
          RCLCPP_INFO(this->get_logger(), "[PARAM] target_alt_m=%.2f", target_alt_m_);
        } else if (p.get_name() == "vz_up") {
          vz_up_ = static_cast<float>(p.as_double());
          RCLCPP_INFO(this->get_logger(), "[PARAM] vz_up=%.2f (akan dipakai negatif di NED)", vz_up_);
        } else if (p.get_name() == "forward_dist_m") {
          forward_dist_m_ = static_cast<float>(p.as_double());
          RCLCPP_INFO(this->get_logger(), "[PARAM] forward_dist_m=%.2f", forward_dist_m_);
        } else if (p.get_name() == "reach_xy_tol") {
          reach_xy_tol_ = static_cast<float>(p.as_double());
          RCLCPP_INFO(this->get_logger(), "[PARAM] reach_xy_tol=%.2f", reach_xy_tol_);
        } else if (p.get_name() == "speed_forward_mps") {
          speed_forward_mps_ = static_cast<float>(p.as_double());
          RCLCPP_INFO(this->get_logger(), "[PARAM] speed_forward_mps=%.2f", speed_forward_mps_);
        } else if (p.get_name() == "slow_k") {
          slow_k_ = static_cast<float>(p.as_double());
          RCLCPP_INFO(this->get_logger(), "[PARAM] slow_k=%.2f", slow_k_);
        } else if (p.get_name() == "vz_hold_max") {
          vz_hold_max_ = static_cast<float>(p.as_double());
          RCLCPP_INFO(this->get_logger(), "[PARAM] vz_hold_max=%.2f", vz_hold_max_);
        }
      }
      rcl_interfaces::msg::SetParametersResult res;
      res.successful = true;
      res.reason = "";
      return res;
    });

  // === Timer ===
  timer_ = create_wall_timer(20ms, std::bind(&ForwardBackward::timer_callback, this));

  RCLCPP_INFO(get_logger(), "ForwardBackward ready. target_alt=%.2fm, forward=%.2fm, v_forward=%.2fm/s",
              target_alt_m_, forward_dist_m_, speed_forward_mps_);
}

void ForwardBackward::local_position_callback(VehicleLocalPosition::ConstSharedPtr msg)
{
  // Posisi & kecepatan (frame NED)
  curr_x_ = msg->x;
  curr_y_ = msg->y;
  curr_z_ = msg->z;

  curr_vx_ = msg->vx;
  curr_vy_ = msg->vy;
  curr_vz_ = msg->vz;

  curr_heading_ = static_cast<float>(msg->heading);

  // Averaging awal untuk baseline
  if (!initial_position_set_) {
    xs_.push_back(curr_x_);
    ys_.push_back(curr_y_);
    zs_.push_back(curr_z_);

    if (xs_.size() >= 50) {
      auto avg = [](const auto &v) { return std::accumulate(v.begin(), v.end(), 0.0) / v.size(); };
      initial_x_   = static_cast<float>(avg(xs_));
      initial_y_   = static_cast<float>(avg(ys_));
      initial_z_   = static_cast<float>(avg(zs_));
      initial_yaw_ = curr_heading_;
      initial_position_set_ = true;
      phase_ = Phase::WAIT_CMD;

      RCLCPP_INFO(this->get_logger(),
                  "Initial avg set: x=%.2f y=%.2f z=%.2f head=%.2f",
                  initial_x_, initial_y_, initial_z_, initial_yaw_);
    }
  }

  if (start_takeoff_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Local Pos curr: [%.2f, %.2f, %.2f] target: [%.2f, %.2f, %.2f] phase=%d",
      curr_x_, curr_y_, curr_z_, target_x_, target_y_, target_z_, static_cast<int>(phase_));
  }
}

void ForwardBackward::timer_callback()
{
  // tarik param runtime (utk start_takeoff)
  this->get_parameter("start_takeoff", start_takeoff_);

  // Kick OFFBOARD + ARM saat siap
  if (initial_position_set_ && start_takeoff_ && !offboard_started_) {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); // PX4: custom mode=offboard
    arm();
    phase_ = Phase::ARMED_SETTLE;
    offboard_started_ = true;
    offboard_setpoint_counter_ = 0;
    RCLCPP_INFO(get_logger(), "[STATE] OFFBOARD+ARM, masuk ARMED_SETTLE");
  }

  // (opsional) auto-land setelah waktu lama (contoh: 60s -> 3000 tick @20ms)
  if (offboard_setpoint_counter_ == 3000) {
    land();
    timer_->cancel();
  }

  publish_offboard_control_mode();
  publish_trajectory_setpoint();
  offboard_setpoint_counter_++;
}

void ForwardBackward::publish_trajectory_setpoint()
{
  px4_msgs::msg::TrajectorySetpoint sp{};
  const float NaN = std::numeric_limits<float>::quiet_NaN();

  if (!initial_position_set_) {
    // Hold sementara di posisi terbaru (belum baseline)
    sp.position  = { curr_x_, curr_y_, curr_z_ };
    sp.yaw       = curr_heading_;
  }
  else if (phase_ == Phase::WAIT_CMD) {
    // siap, nunggu perintah → hold baseline
    target_x_ = initial_x_;
    target_y_ = initial_y_;
    target_z_ = initial_z_; // di bawah masih z down; HOLD baseline sebelum takeoff
    sp.position  = { initial_x_, initial_y_, initial_z_ };
    sp.yaw       = initial_yaw_;
  }
  else if (phase_ == Phase::ARMED_SETTLE) {
    // beri waktu settle ~2 detik setelah ARM
    sp.position  = { initial_x_, initial_y_, initial_z_ };
    sp.yaw       = initial_yaw_;
    if (offboard_setpoint_counter_ > 160) { // ~2s
      phase_ = Phase::TAKEOFF;
      RCLCPP_INFO(get_logger(), "[STATE] TAKEOFF");
    }
  }
  else if (phase_ == Phase::TAKEOFF) {
    // Naikkan altitude pakai velocity (NED: naik = z negatif)
    sp.position  = { initial_x_, initial_y_, NaN };
    sp.velocity  = { NaN, NaN, -std::abs(vz_up_) };
    sp.yaw       = initial_yaw_;

    // Berapa sudah naik? initial_z (down) - curr_z (down) → positif jika naik
    const float climbed = initial_z_ - curr_z_;
    if (climbed >= (target_alt_m_ - 0.05f)) {
      // Hitung titik maju di heading awal (x=N, y=E)
      target_x_ = initial_x_ + forward_dist_m_ * std::cos(initial_yaw_);
      target_y_ = initial_y_ + forward_dist_m_ * std::sin(initial_yaw_);
      target_z_ = initial_z_ - target_alt_m_; // z down agar tinggi = target_alt

      phase_ = Phase::FORWARD;
      RCLCPP_INFO(get_logger(), "[STATE] FORWARD %.2fm menuju (%.2f, %.2f) @alt=%.2f",
                  forward_dist_m_, target_x_, target_y_, target_alt_m_);
    }
  }
  else if (phase_ == Phase::FORWARD) {
    // Bergerak ke (target_x_, target_y_) dengan velocity halus 0.5 m/s (default)
    const float tx = target_x_;
    const float ty = target_y_;
    const float tz = target_z_;

    const float ex = tx - curr_x_;
    const float ey = ty - curr_y_;
    const float ez = tz - curr_z_;
    const float dxy = std::hypot(ex, ey);

    float ux = 0.f, uy = 0.f;
    if (dxy > 1e-3f) {
      ux = ex / dxy;
      uy = ey / dxy;
    }

    float v_cmd = std::min(speed_forward_mps_, slow_k_ * dxy);
    float vx_cmd = v_cmd * ux;
    float vy_cmd = v_cmd * uy;

    // Koreksi kecil di vertikal (Down positif), saturasi agar halus
    float vz_cmd = std::clamp(ez, -vz_hold_max_, vz_hold_max_);

    sp.position = { NaN, NaN, NaN };
    sp.velocity = { vx_cmd, vy_cmd, vz_cmd };
    sp.yaw      = initial_yaw_;

    if (dxy <= reach_xy_tol_) {
      phase_ = Phase::HOLD;
      RCLCPP_INFO(get_logger(), "[STATE] HOLD di titik maju (dXY=%.3f m)", dxy);
    }
  }
  else { // HOLD
    // Tahan di titik target akhir dengan position setpoint (PX4 yang stabilkan)
    sp.position  = { target_x_, target_y_, target_z_ };
    sp.yaw       = initial_yaw_;
  }

  sp.timestamp = get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(sp);
}

void ForwardBackward::publish_offboard_control_mode()
{
  px4_msgs::msg::OffboardControlMode msg{};

  const bool takeoff = (phase_ == Phase::TAKEOFF);
  const bool forward = (phase_ == Phase::FORWARD);

  // Velocity mode saat TAKEOFF & FORWARD; posisi untuk fase lain
  msg.position     = !(takeoff || forward);
  msg.velocity     =  (takeoff || forward);
  msg.acceleration = false;
  msg.attitude     = false;
  msg.body_rate    = false;

  msg.timestamp    = get_clock()->now().nanoseconds() / 1000;
  offboard_control_mode_publisher_->publish(msg);
}

void ForwardBackward::publish_vehicle_command(uint16_t command, float param1, float param2)
{
  px4_msgs::msg::VehicleCommand msg{};
  msg.param1 = param1;
  msg.param2 = param2;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = get_clock()->now().nanoseconds() / 1000;
  vehicle_command_publisher_->publish(msg);
}

void ForwardBackward::arm()
{
  publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
  RCLCPP_INFO(get_logger(), "Arm command sent");
}

void ForwardBackward::land()
{
  publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
  RCLCPP_INFO(get_logger(), "Land command sent (auto-timeout)");
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ForwardBackward>("hover_only");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
