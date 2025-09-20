#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <builtin_interfaces/msg/time.hpp>

#include <array>
#include <cmath>
#include <memory>

class DroneKinematics : public rclcpp::Node
{
    public:
        explicit DroneKinematics(const std::string &name);

    private:
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_;

        std::string odom_frame_, base_link_frame_, topic_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;

        void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
        std::array<double,4> quat_ned_to_enu(double w, double x, double y, double z);
        inline void ned_to_enu_pos(float xn, float ye, float zd, double &xe, double &yn, double &zu);
        builtin_interfaces::msg::Time stamp_now();
};

