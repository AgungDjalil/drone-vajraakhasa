#include "offboard_control/hover_only.hpp"

HoverOnly::HoverOnly(const std::string &name) : Node(name) 
{
    vehicle_local_position_subscriber_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position_v1", rclcpp::SensorDataQoS(),
                                    std::bind(&HoverOnly::local_position_callback, this, _1));
    offboard_control_mode_publisher_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

    offboard_setpoint_counter_ = 0;
    
    timer_ = create_wall_timer(50ms, std::bind(&HoverOnly::timer_callback, this)); 
}

void HoverOnly::local_position_callback(px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr msg)
{
    curr_x_ = msg->x;
    curr_y_ = msg->y;
    curr_z_ = msg->z; 

    if(!initial_position_set_) 
    {
        xs_.push_back(msg->x);
        ys_.push_back(msg->y);
        zs_.push_back(msg->z);

        if (xs_.size() >= 20) 
        {
            auto avg = [](const auto& v){
                return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
            };

            initial_x_ = avg(xs_);
            initial_y_ = avg(ys_);
            initial_z_ = avg(zs_);
            initial_yaw_ = static_cast<float>(msg->heading);
            initial_position_set_ = true;
            
            RCLCPP_INFO(this->get_logger(),
                "Initial avg set: x=%.2f y=%.2f z=%.2f head=%.2f",
                initial_x_, initial_y_, initial_z_);

            phase_ = Phase::TAKEOFF;
        }
    }

    RCLCPP_INFO(this->get_logger(),
                "Local Position \n current -> x: %.2f y: %.2f z: %.2f\n target  -> x: %.2f y: %.2f z: -%.2f",
                msg->x, msg->y, (msg->z - initial_z_), initial_x_, initial_y_, target_alt_m_);
}

void HoverOnly::timer_callback()
{
    if (offboard_setpoint_counter_ == 10) 
    {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        arm();
        if (initial_position_set_) phase_ = Phase::ARMED_SETTLE;
    }

    publish_offboard_control_mode();

    px4_msgs::msg::TrajectorySetpoint sp{};
    const float NaN = std::numeric_limits<float>::quiet_NaN();

    if (!initial_position_set_) 
    {
        sp.position = { curr_x_, curr_y_, curr_z_ };
        sp.yaw = curr_yaw_;
    }
    else if (phase_ == Phase::ARMED_SETTLE) 
    {
        sp.position = { initial_x_, initial_y_, initial_z_ };
        sp.yaw = initial_yaw_;
        if (offboard_setpoint_counter_ > 50) {
            phase_ = Phase::TAKEOFF;
        }
    }
    else if (phase_ == Phase::TAKEOFF) 
    {
        sp.position = { initial_x_, initial_y_, NaN };
        sp.velocity = { NaN, NaN, vz_up_ };
        sp.yaw = initial_yaw_;

        const float climbed = initial_z_ - curr_z_;
        if (climbed >= target_alt_m_ - 0.05f) phase_ = Phase::HOLD;
    }
    else 
    {
        sp.position = { initial_x_, initial_y_, initial_z_ - target_alt_m_ };
        sp.yaw = initial_yaw_;
    }

    sp.timestamp = rclcpp::Clock().now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(sp);
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
    
    msg.position = {initial_x_, initial_y_, -1.0f};
    msg.yaw = initial_z_;
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

void HoverOnly::publish_offboard_control_mode()
{
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = true;
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


