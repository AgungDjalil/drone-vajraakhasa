#ifndef CYLINDer_CROP_HPP
#define CYLINDer_CROP_HPP

#include <rclcpp/rclcpp.hpp>    
#include <sensor_msgs/msg/point_cloud2.hpp>

class CylinderCrop : public rclcpp::Node
{
    public:
        CylinderCrop(const std::string &name);
    
    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

        void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
};

#endif