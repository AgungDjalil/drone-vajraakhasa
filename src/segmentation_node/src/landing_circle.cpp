#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>

class LandingCircleFromSegmented : public rclcpp::Node {
public:
  LandingCircleFromSegmented() : Node("landing_circle_from_segmented") {
    // params
    this->declare_parameter<std::string>("segmented_topic", "/segmented");
    this->declare_parameter<double>("circle_diameter", 7.0);
    this->declare_parameter<double>("circle_height", 0.05);   // tebal marker
    this->declare_parameter<double>("z_offset", 0.0);         // geser marker di sumbu Z (opsional, mis. 0.05)
    this->declare_parameter<int>("min_points", 50);           // minimal titik agar publish

    const auto topic = this->get_parameter("segmented_topic").as_string();
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic, rclcpp::SensorDataQoS(),
      std::bind(&LandingCircleFromSegmented::cb, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("landing_circle", 10);

    RCLCPP_INFO(get_logger(), "Subscribing segmented: %s → publishing /landing_circle", topic.c_str());
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  void cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Ambil param dinamis
    const double dia   = this->get_parameter("circle_diameter").as_double();
    const double h     = this->get_parameter("circle_height").as_double();
    const double zoff  = this->get_parameter("z_offset").as_double();
    const int    nmin  = this->get_parameter("min_points").as_int();

    // Convert ke PCL (cukup ambil XYZ saja; RGB diabaikan)
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    if (cloud.empty() || static_cast<int>(cloud.size()) < nmin) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                           "Segmented cloud too small (%zu < %d), skip", cloud.size(), nmin);
      return;
    }

    // Hitung centroid
    Eigen::Vector4f c;
    pcl::compute3DCentroid(cloud, c);

    // Publish marker (cylinder = lingkaran)
    visualization_msgs::msg::Marker m;
    m.header = msg->header;                 // pakai frame & stamp dari cloud
    m.ns = "landing_zone";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::CYLINDER;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.pose.position.x = c[0];
    m.pose.position.y = c[1];
    m.pose.position.z = c[2] + zoff;        // sedikit diangkat jika mau
    m.pose.orientation.w = 1.0;

    m.scale.x = dia;    // diameter X
    m.scale.y = dia;    // diameter Y
    m.scale.z = h;      // tebal tipis (0.01–0.1)

    // warna hijau transparan
    m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.0f; m.color.a = 0.35f;

    m.lifetime = rclcpp::Duration(0,0);     // persistent
    marker_pub_->publish(m);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LandingCircleFromSegmented>());
  rclcpp::shutdown();
  return 0;
}
