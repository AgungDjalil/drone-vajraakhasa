#ifndef HOVER_ONLY_HPP
#define HOVER_ONLY_HPP

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <atomic>
#include <limits>
#include <vector>
#include <numeric>

using namespace std::chrono;
using namespace std::chrono_literals;
using std::placeholders::_1;

enum class Phase { WAIT_INIT, ARMED_SETTLE, TAKEOFF, HOLD };


class HoverOnly : public rclcpp::Node
{
    public:
        HoverOnly(const std::string &name);
        
        void arm();
        void land();

    private:
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_subscriber_;
        
        uint32_t offboard_setpoint_counter_{0};

        bool   initial_position_set_{false};
        float initial_x_{0}, initial_y_{0}, initial_z_{0}, initial_yaw_{0.0f};
        float curr_x_{0}, curr_y_{0}, curr_z_{0}, curr_yaw_{0.0f};

        std::vector<double> xs_, ys_, zs_;
        Phase phase_{Phase::WAIT_INIT};
        const float target_alt_m_ = 2.0f;
        const float vz_up_        = -0.3f;
        
        void local_position_callback(px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr msg);
        void timer_callback();
        void publish_offboard_control_mode();
        void publish_trajectory_setpoint();
        void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0); 
};

#endif
