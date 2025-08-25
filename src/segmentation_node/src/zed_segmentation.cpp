#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/Geometry>

class ZedSegmentationNode : public rclcpp::Node
{
public:
    ZedSegmentationNode() : Node("zed_segmentation_node")
    {
        // Subscriber ke point cloud ZED
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/zed/zed_node/point_cloud/cloud_registered",
            rclcpp::SensorDataQoS(),
            std::bind(&ZedSegmentationNode::cloudCallback, this, std::placeholders::_1));

        // Publisher hasil segmentasi
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "segmented_cloud", 10);
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty cloud received");
            return;
            }

        RCLCPP_INFO(this->get_logger(),
            "Cloud before downsampling: %zu points",
            cloud->size());

        // Downsampling dengan VoxelGrid
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(0.02f, 0.02f, 0.02f);
        voxel_filter.filter(*cloud);

        RCLCPP_INFO(this->get_logger(),
            "Cloud after downsampling: %zu points",
            cloud->size());

        // PassThrough filter pada sumbu z (vertikal)
        pcl::PassThrough<pcl::PointXYZ> pass;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");  
        pass.setFilterLimits(-8, -0.5);   // ambil titik-titik sekitar bawah kamera
        pass.filter(*cloud_filtered);

        RCLCPP_INFO(this->get_logger(),
            "Cloud Passthrough Filter (lantai): %zu points",
            cloud_filtered->size());

        // Plane segmentation (RANSAC)
        pcl::SACSegmentation<pcl::PointXYZ> plane_segmentation;
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        plane_segmentation.setOptimizeCoefficients(true);
        plane_segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        plane_segmentation.setMethodType(pcl::SAC_RANSAC);
        plane_segmentation.setMaxIterations(1100);
        plane_segmentation.setDistanceThreshold(1);   // toleransi 100 cm
        plane_segmentation.setAxis(Eigen::Vector3f(0,0,1)); // normal lantai ke arah (x,y,z)
        plane_segmentation.setEpsAngle(15.0 * M_PI / 180.0); // toleransi 15 derajat
        plane_segmentation.setInputCloud(cloud_filtered);
        plane_segmentation.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            RCLCPP_WARN(this->get_logger(), "Tidak ada plane ditemukan!");
            return;
        }

        // Extract lantai
        pcl::ExtractIndices<pcl::PointXYZ> extract_indicies;
        pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
        extract_indicies.setInputCloud(cloud_filtered);
        extract_indicies.setIndices(inliers);
        extract_indicies.setNegative(false);
        extract_indicies.filter(*result);

        RCLCPP_INFO(this->get_logger(),
            "Plane ditemukan: %zu points", result->size());
        RCLCPP_INFO(this->get_logger(),
            "Plane coeffs: [%f, %f, %f, %f]",
            coefficients->values[0],
            coefficients->values[1],
            coefficients->values[2],
            coefficients->values[3]);

        // Konversi ke ROS2
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*result, output);
        output.header = msg->header;

        pub_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ZedSegmentationNode>());
    rclcpp::shutdown();
    return 0;
}
