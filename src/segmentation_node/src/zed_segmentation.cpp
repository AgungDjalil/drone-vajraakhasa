#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>

#include <numeric>
#include <vector>

class ZedSegmentation : public rclcpp::Node 
{
    public:
        ZedSegmentation() : Node("zed_segmentation") {
            // I/O topics
            this->declare_parameter<std::string>("input_topic", "/zed/zed_node/point_cloud/cloud_registered");
            this->declare_parameter<std::string>("output_topic", "/segmented");

            // ROI + downsample + denoise
            this->declare_parameter<double>("z_min", -5.0);
            this->declare_parameter<double>("z_max", 5.0);
            this->declare_parameter<double>("leaf_size", 0.1);

            this->declare_parameter<bool>("use_sor", true);
            this->declare_parameter<int>("sor_mean_k", 30);
            this->declare_parameter<double>("sor_std_mul", 1.0);
            this->declare_parameter<double>("ror_radius", 0.25);
            this->declare_parameter<int>("ror_min_neighbors", 8);

            // Plane segmentation params
            this->declare_parameter<int>("max_planes", 2);
            this->declare_parameter<int>("min_plane_inliers", 500);
            this->declare_parameter<double>("plane_distance_thresh", 0.2);
            this->declare_parameter<int>("plane_max_iterations", 200);

            // Cluster cleaning params (untuk “noise kecil” non-inlier)
            this->declare_parameter<double>("cluster_tolerance", 0.2); // meter
            this->declare_parameter<int>("min_cluster_size", 300);

            const auto in  = this->get_parameter("input_topic").as_string();
            const auto out = this->get_parameter("output_topic").as_string();

            sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                in, rclcpp::SensorDataQoS(),
                std::bind(&ZedSegmentation::cloudCb, this, std::placeholders::_1));

            pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(out, 10);

            RCLCPP_INFO(get_logger(), "segmented node subscribed: %s -> %s", in.c_str(), out.c_str());
        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

        void cloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            // ---- Convert input cloud ----
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *cloud);
            if (cloud->empty()) {
                RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 3000, "Empty cloud received");
                return;
            }

            // ---- Parameters ----
            const double z_min   = this->get_parameter("z_min").as_double();
            const double z_max   = this->get_parameter("z_max").as_double();
            const float  leaf    = static_cast<float>(this->get_parameter("leaf_size").as_double());
            const bool   use_sor = this->get_parameter("use_sor").as_bool();

            const int    max_planes        = this->get_parameter("max_planes").as_int();
            const int    min_plane_inliers = this->get_parameter("min_plane_inliers").as_int();
            const double dist_thresh       = this->get_parameter("plane_distance_thresh").as_double();
            const int    max_iters         = this->get_parameter("plane_max_iterations").as_int();

            const double cl_tol            = this->get_parameter("cluster_tolerance").as_double();
            const int    cl_min_size       = this->get_parameter("min_cluster_size").as_int();

            // ---- ROI (PassThrough z) ----
            pcl::PointCloud<pcl::PointXYZ>::Ptr roi(new pcl::PointCloud<pcl::PointXYZ>);
            {
                pcl::PassThrough<pcl::PointXYZ> pass;
                pass.setInputCloud(cloud);
                pass.setFilterFieldName("z");
                pass.setFilterLimits(z_min, z_max);
                pass.filter(*roi);
            }

            if (roi->empty()) {
                RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 3000, "ROI empty");
                return;
            }

            // ---- Downsample (VoxelGrid) ----
            pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>);
            {
                pcl::VoxelGrid<pcl::PointXYZ> vg;
                vg.setInputCloud(roi);
                vg.setLeafSize(leaf, leaf, leaf);
                vg.filter(*down);
            }
            
            if (down->empty()) {
                RCLCPP_WARN(get_logger(), "Empty after VoxelGrid");
                return;
            }

            // ---- Denoise ----
            pcl::PointCloud<pcl::PointXYZ>::Ptr denoised(new pcl::PointCloud<pcl::PointXYZ>);
            if (use_sor) {
                int mean_k = this->get_parameter("sor_mean_k").as_int();
                double stdmul = this->get_parameter("sor_std_mul").as_double();
                pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
                sor.setInputCloud(down);
                sor.setMeanK(mean_k);
                sor.setStddevMulThresh(stdmul);
                sor.filter(*denoised);
            } else {
                double radius = this->get_parameter("ror_radius").as_double();
                int min_n = this->get_parameter("ror_min_neighbors").as_int();
                pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
                ror.setInputCloud(down);
                ror.setRadiusSearch(radius);
                ror.setMinNeighborsInRadius(min_n);
                ror.filter(*denoised);
            }
            if (denoised->empty()) {
                RCLCPP_WARN(get_logger(), "Empty after denoising");
                return;
            }

            // ---- Prepare colored cloud (default RED untuk non-plane) ----
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>);
            colored->reserve(denoised->size());
            for (const auto& p : denoised->points) {
                pcl::PointXYZRGB c;
                c.x = p.x; c.y = p.y; c.z = p.z;
                c.r = 255; c.g = 0; c.b = 0; // RED = non-planar (default)
                colored->push_back(c);
            }

            // ---- Remaining indices (mengacu ke 'denoised') ----
            std::vector<int> remaining_indices(denoised->size());
            std::iota(remaining_indices.begin(), remaining_indices.end(), 0);

            // ---- RANSAC plane segmentation ----
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(dist_thresh);
            seg.setMaxIterations(max_iters);

            for (int plane_id = 0; plane_id < max_planes; ++plane_id) {
                if ((int)remaining_indices.size() < min_plane_inliers) break;

                pcl::PointIndices::Ptr idx(new pcl::PointIndices);
                idx->indices = remaining_indices;

                pcl::ModelCoefficients coeffs;
                pcl::PointIndices inliers;

                seg.setInputCloud(denoised);
                seg.segment(inliers, coeffs);

                if ((int)inliers.indices.size() < min_plane_inliers) break;

                // Plane terbesar pertama = GREEN, berikutnya = ORANGE
                uint8_t R = 0, G = 255, B = 0; // GREEN
                if (plane_id >= 1) { R = 255; G = 165; B = 0; } // ORANGE

                for (int ii : inliers.indices) {
                    colored->points[ii].r = R;
                    colored->points[ii].g = G;
                    colored->points[ii].b = B;
                }

                // Buang inlier dari remaining_indices
                std::vector<char> is_inlier(denoised->size(), 0);
                for (int ii : inliers.indices) is_inlier[ii] = 1;

                std::vector<int> new_remaining;
                new_remaining.reserve(remaining_indices.size());
                for (int idx0 : remaining_indices) {
                    if (!is_inlier[idx0]) new_remaining.push_back(idx0);
                }

                remaining_indices.swap(new_remaining);
            }

            // ---- Publish colored cloud ----
            sensor_msgs::msg::PointCloud2 out;
            pcl::toROSMsg(*colored, out);
            out.header = msg->header; // pertahankan timestamp & frame dari kamera
            pub_->publish(out);
        }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedSegmentation>());
  rclcpp::shutdown();
  return 0;
}
