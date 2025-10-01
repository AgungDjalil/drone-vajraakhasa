#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class OdomTransformer : public rclcpp::Node {
public:
  OdomTransformer() : Node("odom_to_base_link"),
                      tf_buffer_(this->get_clock()),
                      tf_listener_(tf_buffer_)
  {
    in_topic_  = declare_parameter<std::string>("in_odom_topic", "/zed/zed_node/odom");
    out_topic_ = declare_parameter<std::string>("out_odom_topic", "/zed/odom_base_link");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");

    sub_ = create_subscription<nav_msgs::msg::Odometry>(
        in_topic_, 10,
        std::bind(&OdomTransformer::odomCallback, this, std::placeholders::_1));

    pub_ = create_publisher<nav_msgs::msg::Odometry>(out_topic_, 10);

    RCLCPP_INFO(get_logger(), "Transforming %s → %s (target_frame=%s)",
                in_topic_.c_str(), out_topic_.c_str(), target_frame_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      // Ambil TF: target_frame ← child_frame_id (mis. zed_camera_link)
      transform = tf_buffer_.lookupTransform(
          target_frame_, msg->child_frame_id, tf2::TimePointZero);

      // Transform pose
      geometry_msgs::msg::PoseStamped pose_in, pose_out;
      pose_in.header = msg->header;
      pose_in.pose   = msg->pose.pose;
      tf2::doTransform(pose_in, pose_out, transform);

      nav_msgs::msg::Odometry odom_out;
      odom_out.header.stamp    = msg->header.stamp;
      odom_out.header.frame_id = target_frame_;
      odom_out.child_frame_id  = target_frame_;

      odom_out.pose.pose = pose_out.pose;
      odom_out.pose.covariance = msg->pose.covariance;

      // Untuk sederhana: copy twist apa adanya (belum ditransformasi)
      odom_out.twist = msg->twist;

      pub_->publish(odom_out);

    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "Transform failed: %s", ex.what());
    }
  }

  std::string in_topic_, out_topic_, target_frame_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomTransformer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
