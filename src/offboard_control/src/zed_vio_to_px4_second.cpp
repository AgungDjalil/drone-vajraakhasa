#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <array>
#include <cmath>
#include <limits>

using px4_msgs::msg::VehicleOdometry;

class ZedVioToPx4 : public rclcpp::Node {
public:
  ZedVioToPx4() : Node("zed_vio_to_px4")
  {
    in_odom_topic_         = declare_parameter<std::string>("in_odom_topic", "/zed/zed_node/odom");
    out_topic_             = declare_parameter<std::string>("out_topic", "/fmu/in/vehicle_visual_odometry");
    sample_from_msg_stamp_ = declare_parameter<bool>("sample_from_msg_stamp", true);
    use_twist_             = declare_parameter<bool>("use_twist", true);
    camera_downward_       = declare_parameter<bool>("camera_downward", true);
    quality_               = declare_parameter<int>("quality", 100);

    pub_ = create_publisher<VehicleOdometry>(out_topic_, rclcpp::QoS(20).best_effort());
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      in_odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ZedVioToPx4::odomCb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Subscribing ENU odom: %s", in_odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Publishing VehicleOdometry (to EV topic): %s", out_topic_.c_str());
  }

private:
  static inline void posEnuToNed(const geometry_msgs::msg::Point &p, float &x, float &y, float &z) {
    x =  static_cast<float>(p.x);
    y = -static_cast<float>(p.y);
    z = -static_cast<float>(p.z);
  }

  static inline void velEnuToNed(const geometry_msgs::msg::Vector3 &v, float &vx, float &vy, float &vz) {
    vx =  static_cast<float>(v.x);
    vy = -static_cast<float>(v.y);
    vz = -static_cast<float>(v.z);
  }

  static inline std::array<float,4> quatEnuToNed(const geometry_msgs::msg::Quaternion &q_enu)
  {
    tf2::Quaternion qE, qZ, qY, qC, qN;
    tf2::fromMsg(q_enu, qE);
    qZ.setRPY(0, 0, M_PI);
    qY.setRPY(0, M_PI, 0);
    qC = qZ * qY;              // transform ENU -> NED
    qN = qC * qE * qC.inverse();
    return { (float)qN.getW(), (float)qN.getX(), (float)qN.getY(), (float)qN.getZ() };
  }

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    VehicleOdometry out{};

    const uint64_t now_us = (uint64_t)(this->get_clock()->now().nanoseconds() / 1000ULL);
    out.timestamp        = now_us;
    out.timestamp_sample = sample_from_msg_stamp_
      ? (uint64_t)(rclcpp::Time(msg->header.stamp).nanoseconds() / 1000ULL)
      : now_us;

    // ==== Frames (lihat interface kamu): ====
    // pose_frame: VehicleOdometry::POSE_FRAME_NED (1)
    // velocity_frame: VehicleOdometry::VELOCITY_FRAME_NED (1)
    out.pose_frame = VehicleOdometry::POSE_FRAME_NED;
    out.velocity_frame = VehicleOdometry::VELOCITY_FRAME_NED;

    // ==== Position (NED) ====
    posEnuToNed(msg->pose.pose.position, out.position[0], out.position[1], out.position[2]);

    // ==== Orientation (ENU->NED) ====
    geometry_msgs::msg::Quaternion q_in = msg->pose.pose.orientation;
    if (camera_downward_) {
      // jika kamera nadir, perbaiki roll -90° agar yaw body konsisten
      tf2::Quaternion qE; tf2::fromMsg(q_in, qE);
      tf2::Quaternion qFix; qFix.setRPY(-M_PI_2, 0, 0);
      q_in = tf2::toMsg(qFix * qE);
    }
    auto q = quatEnuToNed(q_in);
    out.q = q; // [w,x,y,z]

    // ==== Velocity (optional) ====
    if (use_twist_) {
      velEnuToNed(msg->twist.twist.linear, out.velocity[0], out.velocity[1], out.velocity[2]);
    } else {
      out.velocity[0] = out.velocity[1] = out.velocity[2] = std::numeric_limits<float>::quiet_NaN();
    }

    // Body rates kalau tidak ada sumbernya → NaN
    out.angular_velocity[0] = out.angular_velocity[1] = out.angular_velocity[2] =
        std::numeric_limits<float>::quiet_NaN();

    // Variance konservatif (tune sesuai kualitas VIO)
    out.position_variance[0] = 0.02f; out.position_variance[1] = 0.02f; out.position_variance[2] = 0.03f;
    out.orientation_variance[0] = 0.01f; out.orientation_variance[1] = 0.01f; out.orientation_variance[2] = 0.02f;
    out.velocity_variance[0] = 0.04f; out.velocity_variance[1] = 0.04f; out.velocity_variance[2] = 0.06f;

    out.reset_counter = 0;
    out.quality = static_cast<int8_t>(std::clamp(quality_, 0, 100));

    pub_->publish(out);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<VehicleOdometry>::SharedPtr pub_;

  std::string in_odom_topic_;
  std::string out_topic_;
  bool sample_from_msg_stamp_;
  bool use_twist_;
  bool camera_downward_;
  int  quality_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedVioToPx4>());
  rclcpp::shutdown();
  return 0;
}

