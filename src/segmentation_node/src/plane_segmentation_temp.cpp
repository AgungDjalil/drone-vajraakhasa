#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

class PlaneSegmentation : public rclcpp::Node 
{
    public:
        PlaneSegmentation() : Node("plane_segmentation") // /zed/zed_node/point_cloud/cloud_registered
        {
            this->declare_parameter<std::string>("input_topic", "/zed/zed_node/point_cloud/cloud_registered");
            this->declare_parameter<std::string>("output_topic", "/segmented");
            this->declare_parameter<double>("z_min", -5.0);
            this->declare_parameter<double>("z_max", 5.0);
            this->declare_parameter<double>("leaf_size", 0.03);

            const auto input = this->get_parameter("input_topic").as_string();
            const auto output = this->get_parameter("output_topic").as_string();

            sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                input, rclcpp::SensorDataQoS(),
                std::bind(&PlaneSegmentation::callback, this, std::placeholders::_1)
            );

            pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output, 10);
        }
    
    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

        void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *cloud);

            if(cloud->empty()) return;

            const double z_min   = this->get_parameter("z_min").as_double();
            const double z_max   = this->get_parameter("z_max").as_double();
            const double leaf_size = this->get_parameter("leaf_size").as_double();

            pcl::PointCloud<pcl::PointXYZ>::Ptr region_of_interest_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            {
                pcl::PassThrough<pcl::PointXYZ> pass;
                pass.setInputCloud(cloud);
                pass.setFilterFieldName("z");
                pass.setFilterLimits(z_min, z_max);
                pass.filter(*region_of_interest_cloud);
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr downsampling_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            {
                pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
                voxel_filter.setInputCloud(region_of_interest_cloud);
                voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
                voxel_filter.filter(*downsampling_cloud);
            }

            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*downsampling_cloud, output);
            output.header = msg->header;
            pub_->publish(output);
        }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlaneSegmentation>());
    rclcpp::shutdown();

    return 0;
}