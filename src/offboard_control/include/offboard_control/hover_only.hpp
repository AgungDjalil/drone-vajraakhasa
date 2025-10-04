#pragma once
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include <vector>
#include <numeric>
#include <limits>
#include <functional>
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

class HoverOnly : public rclcpp::Node {
public:
  explicit HoverOnly(const std::string &name);

private:
  // ===== enums & state =====
  enum class Phase { INIT, WAIT_CMD, ARMED_SETTLE, TAKEOFF, HOLD };
  Phase phase_{Phase::INIT};

  // ===== subscribers/publishers =====
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_subscriber_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr      offboard_control_mode_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr       trajectory_setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr           vehicle_command_publisher_;
  rclcpp::TimerBase::SharedPtr                                          timer_;

  // ===== callbacks =====
  void local_position_callback(px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr msg);
  void timer_callback();

  // ===== helpers =====
  void publish_offboard_control_mode();
  void publish_trajectory_setpoint();
  void publish_vehicle_command(uint16_t command, float param1 = 0.f, float param2 = 0.f);
  void arm();
  void land();

  // ===== state vars =====
  float curr_x_{0.f}, curr_y_{0.f}, curr_z_{0.f}, curr_heading_{0.f};
  float initial_x_{0.f}, initial_y_{0.f}, initial_z_{0.f}, initial_yaw_{0.f};

  bool  initial_position_set_{false};
  bool  offboard_started_{false};
  bool  start_takeoff_{false};

  int   offboard_setpoint_counter_{0};

  // averaging buffers
  std::vector<double> xs_, ys_, zs_;

  // ===== params =====
  float target_alt_m_{2.0f}; // ketinggian target (meter)
  float vz_up_{0.6f};        // kecepatan naik (m/s). Akan dipakai negatif (NED)

  // param on-set callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};
