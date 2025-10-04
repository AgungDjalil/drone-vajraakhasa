#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include <memory>
#include <vector>
#include <numeric>
#include <limits>
#include <cmath>
#include <chrono>

class ForwardBackward : public rclcpp::Node
{
public:
  explicit ForwardBackward(const std::string &name);

private:
  using VehicleLocalPosition = px4_msgs::msg::VehicleLocalPosition;

  enum class Phase {
    WAIT_CMD = 0,
    ARMED_SETTLE,
    TAKEOFF,
    FORWARD,
    HOLD
  };

  // === Callbacks ===
  void local_position_callback(VehicleLocalPosition::ConstSharedPtr msg);
  void timer_callback();

  // === Publishers (helpers) ===
  void publish_trajectory_setpoint();
  void publish_offboard_control_mode();
  void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
  void arm();
  void land();

  // === ROS interfaces ===
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_subscriber_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr   trajectory_setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr       vehicle_command_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // === Parameters ===
  float target_alt_m_{2.0f};        // ketinggian target (meter, Up)
  float vz_up_{1.0f};               // kecepatan naik (m/s, dikirim negatif di NED)
  bool  start_takeoff_{false};      // trigger dari parameter

  float forward_dist_m_{1.5f};      // jarak maju (m) mengikuti heading awal
  float reach_xy_tol_{0.10f};       // toleransi "sampai" (m)
  float speed_forward_mps_{0.5f};   // kecepatan maju (m/s)
  float slow_k_{0.8f};              // pelunak kecepatan mendekati target (v=min(vmax, k*jarak))
  float vz_hold_max_{0.6f};         // batas koreksi vertikal saat FORWARD (m/s)

  // === State flags ===
  bool initial_position_set_{false};
  bool offboard_started_{false};

  // === PX4 state (current) ===
  float curr_x_{0.f}, curr_y_{0.f}, curr_z_{0.f}; // N, E, Down
  float curr_vx_{0.f}, curr_vy_{0.f}, curr_vz_{0.f};
  float curr_heading_{0.f}; // yaw (rad, terhadap North)

  // === Initial baseline ===
  float initial_x_{0.f}, initial_y_{0.f}, initial_z_{0.f};
  float initial_yaw_{0.f};

  // === Target point for HOLD/FORWARD ===
  float target_x_{0.f}, target_y_{0.f}, target_z_{0.f};

  // === Averaging buffers ===
  std::vector<double> xs_, ys_, zs_;

  // === Phase & counter ===
  Phase phase_{Phase::WAIT_CMD};
  uint32_t offboard_setpoint_counter_{0};
};

int main(int argc, char* argv[]);
