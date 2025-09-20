#include "offboard_control/hover_only.hpp"
#include <rcl_interfaces/msg/set_parameters_result.hpp>

HoverOnly::HoverOnly(const std::string &name) : Node(name)
{
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

  vehicle_local_position_subscriber_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", qos,
      std::bind(&HoverOnly::local_position_callback, this, _1));

  offboard_control_mode_publisher_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
  trajectory_setpoint_publisher_   = create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
  vehicle_command_publisher_       = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

  // declare params
  target_alt_m_  = declare_parameter<float>("target_alt_m", 2.0f);
  vz_up_         = declare_parameter<float>("vz_up", 1.0f);            // positif; akan di-negatifkan untuk NED
  declare_parameter<bool>("start_takeoff", false);    // trigger manual

  // on-set parameter callback
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
            }
        }
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = true;
        res.reason = "";
        return res;
    });


  timer_ = create_wall_timer(20ms, std::bind(&HoverOnly::timer_callback, this));
}

void HoverOnly::local_position_callback(px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr msg)
{
  curr_x_ = msg->x;
  curr_y_ = msg->y;
  curr_z_ = msg->z;
  curr_heading_ = static_cast<float>(msg->heading);

  if (!initial_position_set_) {
    xs_.push_back(curr_x_);
    ys_.push_back(curr_y_);
    zs_.push_back(curr_z_);

    if (xs_.size() >= 10) { // cukup 10 sampel biar cepat tapi halus
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

  if (start_takeoff_)
    RCLCPP_INFO(this->get_logger(),
                "Local Position \n current -> x: %.2f y: %.2f z: %.2f\n target  -> x: %.2f y: %.2f z: -%.2f",
                msg->x, msg->y, msg->z, initial_x_, initial_y_, target_alt_m_);
}

void HoverOnly::timer_callback()
{
  // Baca parameter terbaru setiap tick (polling)
  // Pastikan param sudah dideclare di ctor (kamu sudah lakukan)
  this->get_parameter("start_takeoff", start_takeoff_);
  // Mulai OFFBOARD + ARM sekali setelah baseline siap & user trigger
  if (initial_position_set_ && start_takeoff_ && !offboard_started_) {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); // OFFBOARD
    arm();
    phase_ = Phase::ARMED_SETTLE;   // settle sebentar sebelum naik
    offboard_started_ = true;
    RCLCPP_INFO(get_logger(), "[STATE] OFFBOARD+ARM, masuk ARMED_SETTLE");
  }

  // Auto land opsional (~60s)
  if (offboard_setpoint_counter_ == 3000) {
    land();
    timer_->cancel();
  }

  publish_offboard_control_mode();
  publish_trajectory_setpoint();
  offboard_setpoint_counter_++;
}

void HoverOnly::publish_trajectory_setpoint()
{
  px4_msgs::msg::TrajectorySetpoint sp{};
  const float NaN = std::numeric_limits<float>::quiet_NaN();

  // sebelum avg siap → tahan di posisi & yaw sekarang
  if (!initial_position_set_) {
    sp.position  = { curr_x_, curr_y_, curr_z_ };
    sp.yaw       = curr_heading_; // kunci yaw agar tidak geser
  }
  else if (phase_ == Phase::WAIT_CMD) {
    // baseline siap, nunggu perintah → hold di baseline
    sp.position  = { initial_x_, initial_y_, initial_z_ };
    sp.yaw       = initial_yaw_;
  }
  else if (phase_ == Phase::ARMED_SETTLE) {
    // berikan ~2 detik untuk settle setelah ARM
    sp.position  = { initial_x_, initial_y_, initial_z_ };
    sp.yaw       = initial_yaw_;
    if (offboard_setpoint_counter_ > 160) { // ~2s
      phase_ = Phase::TAKEOFF;
      RCLCPP_INFO(get_logger(), "[STATE] TAKEOFF");
    }
  }
  else if (phase_ == Phase::TAKEOFF) {
    // kunci x,y; naikkan z pakai velocity (NED: naik = z negatif)
    sp.position  = { initial_x_, initial_y_, NaN };
    sp.velocity  = { NaN, NaN, -std::abs(vz_up_) };
    sp.yaw       = initial_yaw_;
    const float climbed = initial_z_ - curr_z_; // positif kalau naik
    if (climbed >= (target_alt_m_ - 0.05f)) {
      phase_ = Phase::HOLD;
      RCLCPP_INFO(get_logger(), "[STATE] HOLD @ %.2fm", target_alt_m_);
    }
  }
  else { // HOLD
    sp.position  = { initial_x_, initial_y_, initial_z_ - target_alt_m_ };
    sp.yaw       = initial_yaw_;
  }

  sp.timestamp = get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(sp);
}

void HoverOnly::publish_offboard_control_mode()
{
  px4_msgs::msg::OffboardControlMode msg{};

  const bool warmup  = !initial_position_set_;
  const bool settle  = (phase_ == Phase::ARMED_SETTLE);
  const bool takeoff = (phase_ == Phase::TAKEOFF);
  const bool hold    = (phase_ == Phase::HOLD);
  (void)hold;

  msg.position     = true;         // selalu kontrol posisi
  msg.velocity     = takeoff;      // velocity hanya aktif saat TAKEOFF
  msg.acceleration = false;
  msg.attitude     = false;
  msg.body_rate    = false;

  msg.timestamp    = get_clock()->now().nanoseconds() / 1000;
  offboard_control_mode_publisher_->publish(msg);
}

void HoverOnly::publish_vehicle_command(uint16_t command, float param1, float param2)
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

void HoverOnly::arm()
{
  publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
  RCLCPP_INFO(get_logger(), "Arm command sent");
}

void HoverOnly::land()
{
  publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
  RCLCPP_INFO(get_logger(), "Land command sent (after 60s)");
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HoverOnly>("hover_only");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
