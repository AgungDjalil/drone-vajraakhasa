#include "offboard_control/safety_landing.hpp"

using namespace std::chrono_literals;

SafetyLanding::SafetyLanding(const std::string &name) : Node(name)
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    vehicle_local_position_subscription_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position_v1", qos,
        std::bind(&SafetyLanding::local_position_callback, this, _1));
    land_detect_subcription_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
      "/fmu/out/vehicle_land_detected", qos,
      std::bind(&SafetyLanding::land_detected_callback, this, _1));

    offboard_control_mode_publisher_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    
    declare_parameter<bool>("start_takeoff", false);
    declare_parameter<bool>("landing", false);
    vz_down_max_ = declare_parameter<float>("vz_down_max", 0.5f);

    param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&SafetyLanding::param_callback, this, _1)
    );

    timer_ = create_wall_timer(20ms, std::bind(&SafetyLanding::timer_callback, this));

    RCLCPP_INFO(get_logger(), "target_alt=%.2fm, forward=%.2fm, v_forward=%.2fm/s",
                              target_altitude_m_, forward_distance_m_, speed_forward_mps_);
}

void SafetyLanding::land_detected_callback(px4_msgs::msg::VehicleLandDetected::ConstSharedPtr msg) 
{
  landed_.store(msg->landed);
  ground_contact_.store(msg->ground_contact);
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "[LD] landed=%d ground_contact=%d", (int)landed_.load(), (int)ground_contact_.load());
}

void SafetyLanding::timer_callback()
{
  get_parameter("start_takeoff", start_takeoff_);
  
  if (phase_ != Phase::DISARM)
  {
    publish_offboard_control_mode();
    if (phase_ != Phase::IDLE)
    {
      publish_trajectory_setpoint();
      offboard_setpoint_counter_++;
    } else if (phase_ == Phase::IDLE)
    {
      // publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
      phase_ = Phase::DISARM;
    }
  }

  if(start_takeoff_ && initial_position_set_ && !offboard_started_)
  {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
    offboard_started_ = true;
    phase_ = Phase::ARMED_SETTLE;
    offboard_setpoint_counter_ = 0;
    RCLCPP_INFO(get_logger(), "[STATE] OFFBOARD+ARM");
  }
}

void SafetyLanding::publish_trajectory_setpoint()
{
  px4_msgs::msg::TrajectorySetpoint sp{};
  const float NaN = std::numeric_limits<float>::quiet_NaN();

  if (!initial_position_set_) 
  {
    sp.position  = { curr_x_, curr_y_, curr_z_ };
    sp.yaw       = curr_heading_;
  }
  else if (phase_ == Phase::WAIT_CMD) 
  {
    target_x_ = initial_x_;
    target_y_ = initial_y_;
    target_z_ = initial_z_;
    sp.position  = { initial_x_, initial_y_, initial_z_ };
    sp.yaw       = initial_yaw_;
  }
  else if (phase_ == Phase::ARMED_SETTLE) 
  {
    sp.position  = { initial_x_, initial_y_, initial_z_ };
    sp.yaw       = initial_yaw_;
    if (offboard_setpoint_counter_ > 200) {
      phase_ = Phase::TAKEOFF;
      RCLCPP_WARN(get_logger(), "[STATE] TAKEOFF");
    }
  }
  else if (phase_ == Phase::TAKEOFF) 
  {
    const float climbed   = initial_z_ - curr_z_;
    const float climb_rem = std::max(0.0f, target_altitude_m_ - climbed);

    const float k_z   = 0.8f;       
    float v_up_cmd    = std::min(vz_up_, k_z * climb_rem);
    v_up_cmd          = std::max(0.0f, v_up_cmd);

    const float z_tol = 0.05f;
    if (climb_rem < z_tol) v_up_cmd = 0.0f;

    sp.position = { initial_x_, initial_y_, NaN };
    sp.velocity = { NaN, NaN, -v_up_cmd };
    sp.yaw      = initial_yaw_;

    if (climb_rem <= z_tol) {
      target_x_ = initial_x_ + forward_distance_m_ * std::cos(initial_yaw_);
      target_y_ = initial_y_ + forward_distance_m_ * std::sin(initial_yaw_);
      target_z_ = initial_z_ - target_altitude_m_;

      phase_ = Phase::FORWARD;
      RCLCPP_WARN(get_logger(), "[STATE] FORWARD %.2fm menuju (%.2f, %.2f) @alt=%.2f",
                  forward_distance_m_, target_x_, target_y_, target_altitude_m_);
    }
  }
  else if (phase_ == Phase::FORWARD) 
  {
    const float tx = target_x_;
    const float ty = target_y_;
    const float tz = target_z_;

    const float ex = tx - curr_x_;
    const float ey = ty - curr_y_;
    const float ez = tz - curr_z_;
    const float dxy = std::hypot(ex, ey);

    float ux = 0.f, uy = 0.f;
    if (dxy > 1e-3f) 
    {
      ux = ex / dxy;
      uy = ey / dxy;
    }

    float v_cmd = std::min(speed_forward_mps_, slow_k_ * dxy);
    float vx_cmd = v_cmd * ux;
    float vy_cmd = v_cmd * uy;

    float vz_cmd = std::clamp(ez, -vz_hold_max_, vz_hold_max_);

    sp.position = { NaN, NaN, NaN };
    sp.velocity = { vx_cmd, vy_cmd, vz_cmd };
    sp.yaw      = initial_yaw_;

    if (dxy <= reach_xy_tol_) {
      phase_ = Phase::HOLD;
      RCLCPP_WARN(get_logger(), "[STATE] HOLD di titik maju (dXY=%.3f m)", dxy);
    }
  }
  else if (phase_ == Phase::LANDING)
  {
    const float NaN = std::numeric_limits<float>::quiet_NaN();

    const float land_rem = initial_z_ - curr_z_;

    const float k_z = 0.8f;
    float v_down_cmd = std::min(vz_down_max_, std::max(0.0f, k_z * land_rem));

    sp.position = { target_x_, target_y_, NaN };     // XY hold, Z pakai velocity
    sp.velocity = { 0.001, 0.001, v_down_cmd };      // NED: turun = positif
    sp.yaw      = curr_heading_;

    const bool px4_landed = landed_.load();
    const bool touch_and_still = ground_contact_.load() && std::fabs(curr_vz_) < 0.10f;
    const bool below_threshold = (land_rem < 0.05f);   // < 5 cm dari baseline

    if (below_threshold) 
    {
        sp.position = { NaN, NaN, curr_z_ };
        sp.velocity = { 0.0001, 0.0001, 0.1 };
        phase_ = Phase::IDLE;
    }
  } 
  else if (phase_ == Phase::HOLD)
  {
    sp.position  = { target_x_, target_y_, target_z_ };
    sp.yaw       = initial_yaw_;
  }

  sp.timestamp = get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(sp);
}

void SafetyLanding::publish_offboard_control_mode()
{
  px4_msgs::msg::OffboardControlMode msg{};

  const bool takeoff = (phase_ == Phase::TAKEOFF);
  const bool forward = (phase_ == Phase::FORWARD);
  const bool landing = (phase_ == Phase::LANDING);

  msg.position = !(takeoff || forward) || landing;
  msg.velocity =  (takeoff || forward) || landing;  
  msg.acceleration = false;
  msg.attitude     = false;
  msg.body_rate    = false;

  msg.timestamp    = get_clock()->now().nanoseconds() / 1000;
  offboard_control_mode_publisher_->publish(msg);
}

void SafetyLanding::publish_vehicle_command(uint16_t command, float param1, float param2)
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

void SafetyLanding::local_position_callback(px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr msg)
{
  curr_x_ = msg->x;
  curr_y_ = msg->y;
  curr_z_ = msg->z;

  curr_vx_ = msg->vx;
  curr_vy_ = msg->vy;
  curr_vz_ = msg->vz;

  curr_heading_ = static_cast<float>(msg->heading);

  if (!initial_position_set_) 
  {
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

      RCLCPP_INFO(get_logger(),
                  "Initial avg set: x=%.2f y=%.2f z=%.2f head=%.2f",
                  initial_x_, initial_y_, initial_z_, initial_yaw_);
    }
  }

  if (start_takeoff_) 
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "Local Pos curr: [%.2f, %.2f, %.2f] target: [%.2f, %.2f, %.2f] phase=%d",
      curr_x_, curr_y_, curr_z_, target_x_, target_y_, target_z_, static_cast<int>(phase_));
  }
}

rcl_interfaces::msg::SetParametersResult SafetyLanding::param_callback(const std::vector<rclcpp::Parameter> &params)
{
  for (const auto &p : params) 
  {
    if (p.get_name() == "start_takeoff") 
    {
      start_takeoff_ = p.as_bool();
      if (start_takeoff_ && initial_position_set_) {
        phase_ = Phase::WAIT_CMD;
        RCLCPP_INFO(get_logger(),
          "[CMD] start_takeoff=true diterima (siap OFFBOARD saat stream stabil)");
      }
    } else if (p.get_name() == "landing")
    {
      landing_ = p.as_bool();
      phase_    = Phase::LANDING;
    }
  }

  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;
  res.reason = "";
  return res;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SafetyLanding>("safety_landing");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
