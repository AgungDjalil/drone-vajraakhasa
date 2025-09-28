#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;

class SafetyLanding : public rclcpp::Node
{
    public:
        explicit SafetyLanding(const std::string &name);
    
    private:

        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_subscription_;
        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
        rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detect_subcription_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

        float target_altitude_m_{2.0f}; // (m)
        float vz_up_{1.0}; // (m/s)
        float forward_distance_m_{2.0};
        float reach_xy_tol_{0.10}; // (m)
        float speed_forward_mps_{1.0}; // (m/s)
        float slow_k_{0.8f}; // pelunak kecepatan mendekati target (v=min(vmax, k*jarak)), lebih kecil lebih smooth (0.2 - 1.0)
        float vz_hold_max_{0.6f}; // batas koreksi vertikal saat FORWARD (m/s)
        float vz_down_max_{0.2};
        uint32_t offboard_setpoint_counter_{0};
        bool start_takeoff_{false};
        bool initial_position_set_{false};
        bool offboard_started_{false};
        bool landing_{false};
        bool landing_circle_started_{false}; 
        
        // current position
        float curr_x_{0.f}, curr_y_{0.f}, curr_z_{0.f}; // N, E, Down
        float curr_vx_{0.f}, curr_vy_{0.f}, curr_vz_{0.f};
        float curr_heading_{0.f}; // yaw (rad, terhadap North)
        
        // initial position
        float initial_x_{0.f}, initial_y_{0.f}, initial_z_{0.f};
        float initial_yaw_{0.f};
        
        // Target position
        float target_x_{0.f}, target_y_{0.f}, target_z_{0.f};
        
        // === Averaging buffers ===
        std::vector<double> xs_, ys_, zs_;

        std::atomic<bool> landed_{false}, ground_contact_{false};

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::string world_frame_{"odom"};
        std::string safety_frame_{"safety_point"};
        float speed_safety_mps_{0.6f};  
        
        enum class Phase 
        {
            WAIT_CMD = 0,
            ARMED_SETTLE,
            TAKEOFF,
            FORWARD,
            HOLD,
            LANDING,
            IDLE,
            DISARM,
            GOTO_SAFETY
        };
        Phase phase_{Phase::WAIT_CMD};
        
        rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &params);
        void land_detected_callback(px4_msgs::msg::VehicleLandDetected::ConstSharedPtr msg);
        void local_position_callback(px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr msg);
        void timer_callback();
        void publish_trajectory_setpoint();
        void publish_offboard_control_mode();
        void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
};