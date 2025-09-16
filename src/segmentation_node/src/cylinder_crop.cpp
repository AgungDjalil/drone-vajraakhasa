#include "segmentation_node/cylinder_crop.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

using std::placeholders::_1;

CylinderCrop::CylinderCrop(const std::string &name) : Node(name)
{
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          "/zed/zed_node/point_cloud/cloud_registered", rclcpp::SensorDataQoS(),
          std::bind(&CylinderCrop::callback, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/cylinder_crop", 10);
}

void CylinderCrop::callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    if(cloud->empty())
        return;

    {
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(0.05, 0.05, 0.05);
        voxel_filter.filter(*cloud);
    }

    {
      // === Cylinder crop (axis = Z): radius di XY, tinggi dibatasi Z ===
      // TODO: ganti konstanta di bawah jadi parameter kalau mau dinamis
      const float cx = 0.0f;            // center X
      const float cy = 0.0f;            // center Y
      const float outer_radius = 1.5f;  // meter
      const float inner_radius = 0.0f;  // 0 = solid cylinder
      const bool  use_z_filter = true;  // batasi tinggi di Z
      const float z_min = -1.0f;        // meter
      const float z_max =  1.0f;        // meter

      const float r2o = outer_radius * outer_radius;
      const float r2i = inner_radius > 0.0f ? inner_radius * inner_radius : 0.0f;

      auto cropped = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      cropped->reserve(cloud->size());

      for (const auto& p : cloud->points)
      {
        if (use_z_filter && (p.z < z_min || p.z > z_max)) continue;

        const float dx = p.x - cx;
        const float dy = p.y - cy;
        const float d2 = dx*dx + dy*dy;           // jarak^2 ke pusat (cx,cy) di bidang XY
        if (d2 >= r2i && d2 <= r2o)
          cropped->push_back(p);
      }

      cropped->width  = static_cast<uint32_t>(cropped->size());
      cropped->height = 1;
      cropped->is_dense = false;

      cloud.swap(cropped); // pakai hasil crop
    }

    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*cloud, out_msg);
    out_msg.header = msg->header;
    pub_->publish(out_msg);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CylinderCrop>("cylinder_crop");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
