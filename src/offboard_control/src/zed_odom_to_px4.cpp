#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <Eigen/Dense>

using px4_msgs::msg::VehicleOdometry;

class ZedOdomToPx4 : public rclcpp::Node {
public:
  ZedOdomToPx4() : Node("zed_odom_to_px4")
  {
    in_odom_topic_  = declare_parameter<std::string>("in_odom_topic", "/zed/zed_node/odom");
    out_topic_      = declare_parameter<std::string>("out_topic", "/fmu/in/vehicle_visual_odometry");
    use_variance_   = declare_parameter<bool>("use_variance", true);
    sample_from_msg_stamp_ = declare_parameter<bool>("sample_from_msg_stamp", true);

    pub_ = create_publisher<VehicleOdometry>(out_topic_, rclcpp::QoS(20).best_effort());
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      in_odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ZedOdomToPx4::cb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Subscribing: %s", in_odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Publishing : %s", out_topic_.c_str());
  }

private:
  void cb(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    VehicleOdometry vo{};

    // ===== Timestamps (µs) =====
    const uint64_t now_us = static_cast<uint64_t>(this->now().nanoseconds() / 1000ULL);
    vo.timestamp        = now_us;
    vo.timestamp_sample = sample_from_msg_stamp_
      ? static_cast<uint64_t>(rclcpp::Time(odom->header.stamp).nanoseconds() / 1000ULL)
      : now_us;

    // ===== Pose (ENU -> NED) =====
    vo.pose_frame = VehicleOdometry::POSE_FRAME_NED;

    const double xE = odom->pose.pose.position.x;
    const double yE = odom->pose.pose.position.y;
    const double zE = odom->pose.pose.position.z;
    vo.position[0] = static_cast<float>(yE);
    vo.position[1] = static_cast<float>(xE);
    vo.position[2] = static_cast<float>(-zE);

    const auto &q = odom->pose.pose.orientation;
    Eigen::Quaterniond q_enu(q.w, q.x, q.y, q.z);
    Eigen::Matrix3d R_enu = q_enu.toRotationMatrix();
    static const Eigen::Matrix3d E = (Eigen::Matrix3d() <<
      0, 1, 0,
      1, 0, 0,
      0, 0, -1).finished();
    Eigen::Matrix3d R_ned = E * R_enu * E;
    Eigen::Quaterniond q_ned(R_ned); q_ned.normalize();

    vo.q[0] = static_cast<float>(q_ned.w());
    vo.q[1] = static_cast<float>(q_ned.x());
    vo.q[2] = static_cast<float>(q_ned.y());
    vo.q[3] = static_cast<float>(q_ned.z());

    // ===== Velocity (BODY: FLU -> FRD) =====
    // nav_msgs/Odometry::twist biasanya di child frame (base/camera) -> di ROS: FLU
    // PX4 BODY_FRD: Forward-Right-Down -> hanya flipping tanda Y & Z.
    const auto &vE = odom->twist.twist.linear;
    const auto &wE = odom->twist.twist.angular;

    vo.velocity_frame = VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
    vo.velocity[0] = static_cast<float>(vE.x);   // Forward stays Forward
    vo.velocity[1] = static_cast<float>(-vE.y);  // Left -> Right
    vo.velocity[2] = static_cast<float>(-vE.z);  // Up   -> Down

    vo.angular_velocity[0] = static_cast<float>(wE.x);   // roll  about X
    vo.angular_velocity[1] = static_cast<float>(-wE.y);  // pitch about Y
    vo.angular_velocity[2] = static_cast<float>(-wE.z);  // yaw   about Z

    // ===== Variances =====
    if (use_variance_) {
      vo.position_variance[0]   = static_cast<float>(odom->pose.covariance[0]);
      vo.position_variance[1]   = static_cast<float>(odom->pose.covariance[7]);
      vo.position_variance[2]   = static_cast<float>(odom->pose.covariance[14]);
      vo.orientation_variance[0]= static_cast<float>(odom->pose.covariance[21]);
      vo.orientation_variance[1]= static_cast<float>(odom->pose.covariance[28]);
      vo.orientation_variance[2]= static_cast<float>(odom->pose.covariance[35]);
      vo.velocity_variance[0]   = static_cast<float>(odom->twist.covariance[0]);
      vo.velocity_variance[1]   = static_cast<float>(odom->twist.covariance[7]);
      vo.velocity_variance[2]   = static_cast<float>(odom->twist.covariance[14]);
    } else {
      vo.position_variance[0] = vo.position_variance[1] = vo.position_variance[2] = -1.f;
      vo.orientation_variance[0] = vo.orientation_variance[1] = vo.orientation_variance[2] = -1.f;
      vo.velocity_variance[0] = vo.velocity_variance[1] = vo.velocity_variance[2] = -1.f;
    }

    vo.reset_counter = 0;
    vo.quality = 0;

    pub_->publish(vo);
  }

  std::string in_odom_topic_;
  std::string out_topic_;
  bool use_variance_{true};
  bool sample_from_msg_stamp_{true};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<VehicleOdometry>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedOdomToPx4>());
  rclcpp::shutdown();
  return 0;
}
