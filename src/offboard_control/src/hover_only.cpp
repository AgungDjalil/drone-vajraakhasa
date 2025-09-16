#include "offboard_control/hover_only.hpp"

HoverOnly::HoverOnly(const std::string &name) : Node(name) 
{
    offboard_control_mode_publisher_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

    offboard_setpoint_counter_ = 0;
    
    timer_ = create_wall_timer(100ms, std::bind(&HoverOnly::timer_callback, this)); 
}

void HoverOnly::timer_callback()
{
    if(offboard_setpoint_counter_ == 10)
    {
       publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
       arm();
    }

    if(offboard_setpoint_counter_ == 600)
    {
        land();
        timer_->cancel();
    }
    
    publish_offboard_control_mode();
    publish_trajectory_setpoint();

    offboard_setpoint_counter_++;
}

void HoverOnly::land()
{
  publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
  RCLCPP_INFO(get_logger(), "Land command sent (after 60s)");
}

void HoverOnly::publish_trajectory_setpoint()
{
    px4_msgs::msg::TrajectorySetpoint msg{};

    const float NaN = std::numeric_limits<float>::quiet_NaN();

    msg.position = {NaN, NaN, -1.0f};
    msg.yaw = NaN;
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

void HoverOnly::publish_offboard_control_mode()
{
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
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
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(get_logger(), "Arm command send");
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HoverOnly>("hover_only");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


