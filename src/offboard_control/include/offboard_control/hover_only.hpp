#ifndef HOVER_ONLY_HPP
#define HOVER_ONLY_HPP

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;

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
        
        std::atomic<uint64_t> offboard_setpoint_counter_;

        uint64_t offboard_control_setpoint_counter_;
        
        void timer_callback();
        void publish_offboard_control_mode();
        void publish_trajectory_setpoint();
        void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0); 
};

#endif
