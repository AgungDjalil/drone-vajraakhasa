#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

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

            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            {
                pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
                ne.setInputCloud(downsampling_cloud);
                ne.setSearchMethod(tree);
                ne.setKSearch(15);
                ne.compute(*normals);
            }

            // RANSAC from normals
            pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> segmented;
            segmented.setOptimizeCoefficients(true);
            segmented.setModelType(pcl::SACMODEL_NORMAL_PLANE);
            segmented.setMethodType(pcl::SAC_RANSAC);
            segmented.setNormalDistanceWeight(0.05f);
            segmented.setDistanceThreshold(0.10f);
            segmented.setMaxIterations(800);

            // >>> kamera menatap ke bawah: normal tanah ~ -Z <<<
            segmented.setAxis(Eigen::Vector3f(0.0f, 0.0f, -1.0f));
            segmented.setEpsAngle(0.35f); // ~20°

            segmented.setInputCloud(downsampling_cloud);
            segmented.setInputNormals(normals);

            pcl::ModelCoefficients coeffs;
            pcl::PointIndices inliers;
            segmented.segment(inliers, coeffs);

            if (inliers.indices.empty()) {
                RCLCPP_WARN(get_logger(), "No plane found");
                return;
            }

            // Ambil hanya titik bidang
            pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(downsampling_cloud);
            extract.setIndices(std::make_shared<pcl::PointIndices>(inliers));
            extract.setNegative(false); // false = keep only inliers
            extract.filter(*plane_cloud);

            // Publish hanya bidang
            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*plane_cloud, output);
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