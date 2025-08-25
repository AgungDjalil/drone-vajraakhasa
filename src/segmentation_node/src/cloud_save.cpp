#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class CloudSaver : public rclcpp::Node {
public:
    CloudSaver() : Node("cloud_saver"), saved(false) {
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/zed/zed_node/point_cloud/cloud_registered",   // ganti sesuai topic ZED kamu
            rclcpp::SensorDataQoS(),
            std::bind(&CloudSaver::callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Waiting for one point cloud frame...");
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (saved) return;  // jangan simpan lebih dari sekali

        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::fromROSMsg(*msg, cloud);

        std::string filename = "/home/ubuntu22/Projects/drone_vajraakhasa/src/segmentation_node/clouds/pertama.pcd";
        if (pcl::io::savePCDFileBinary(filename, cloud) == 0) {
            RCLCPP_INFO(this->get_logger(), "Saved ONE frame to %s", filename.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to save cloud!");
        }

        saved = true;
        // hentikan node setelah menyimpan
        rclcpp::shutdown();
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    bool saved;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudSaver>());
    rclcpp::shutdown();
    return 0;
}
