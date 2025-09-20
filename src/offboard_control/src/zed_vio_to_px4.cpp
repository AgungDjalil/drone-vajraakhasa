#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <functional>   // std::bind, std::placeholders
#include <cmath>        // M_PI, M_PI_2

using px4_msgs::msg::VehicleOdometry;

class ZedVioToPx4 : public rclcpp::Node {
public:
  ZedVioToPx4() : Node("zed_vio_to_px4")
  {
    in_odom_topic_ = declare_parameter<std::string>("in_odom_topic", "/zed/zed_node/odom");
    out_topic_     = declare_parameter<std::string>("out_topic", "/fmu/in/vehicle_visual_odometry");
    use_variance_  = declare_parameter<bool>("use_variance", false);
    sample_from_msg_stamp_ = declare_parameter<bool>("sample_from_msg_stamp", true);

    pub_ = create_publisher<VehicleOdometry>(out_topic_, rclcpp::QoS(20).best_effort());

    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      in_odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ZedVioToPx4::odomCb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Sub: %s  →  Pub: %s (VehicleOdometry)",
                in_odom_topic_.c_str(), out_topic_.c_str());
  }

private:
  // ENU (x E, y N, z U) → NED (x N, y E, z D)
  static inline void enuToNedVec(const double enu[3], float ned[3]) {
    ned[0] = static_cast<float>(enu[1]);   // N
    ned[1] = static_cast<float>(enu[0]);   // E
    ned[2] = static_cast<float>(-enu[2]);  // D
  }

  // Quaternion ENU → NED: q_ned = Rz(+90°)*Rx(180°) * q_enu
  static inline void enuToNedQuat(const tf2::Quaternion &q_enu, float q_out[4]) {
    tf2::Quaternion q_conv; 
    q_conv.setRPY(M_PI, 0.0, M_PI_2); // roll=pi, pitch=0, yaw=pi/2
    tf2::Quaternion q_ned = q_conv * q_enu; 
    q_ned.normalize();
    q_out[0] = static_cast<float>(q_ned.w());
    q_out[1] = static_cast<float>(q_ned.x());
    q_out[2] = static_cast<float>(q_ned.y());
    q_out[3] = static_cast<float>(q_ned.z());
  }

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    VehicleOdometry vo{};

    const uint64_t now_us = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000ULL);
    vo.timestamp = now_us;
    vo.timestamp_sample = sample_from_msg_stamp_
      ? static_cast<uint64_t>(rclcpp::Time(msg->header.stamp).nanoseconds() / 1000ULL)
      : now_us;

    vo.pose_frame     = VehicleOdometry::POSE_FRAME_NED;
    vo.velocity_frame = VehicleOdometry::VELOCITY_FRAME_NED;

    // Posisi ENU → NED
    double p_enu[3] = {
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z
    };
    enuToNedVec(p_enu, vo.position.data());

    // Orientasi ENU → NED (urutan q: [w,x,y,z] untuk PX4)
    const auto &q = msg->pose.pose.orientation;
    tf2::Quaternion q_enu(q.x, q.y, q.z, q.w);
    enuToNedQuat(q_enu, vo.q.data());

    // Velocity linear ENU → NED
    double v_enu[3] = {
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.linear.z
    };
    enuToNedVec(v_enu, vo.velocity.data());

    // Angular velocity (body rates) ENU → NED
    double w_enu[3] = {
      msg->twist.twist.angular.x,
      msg->twist.twist.angular.y,
      msg->twist.twist.angular.z
    };
    enuToNedVec(w_enu, vo.angular_velocity.data());

    // Variance 3-elemen (pos/orient/vel). Isi NAN jika tidak dipakai.
    if (use_variance_) {
      vo.position_variance     = {0.02f, 0.02f, 0.05f};
      vo.orientation_variance  = {0.01f, 0.01f, 0.02f};
      vo.velocity_variance     = {0.10f, 0.10f, 0.15f};
    } else {
      vo.position_variance     = {NAN, NAN, NAN};
      vo.orientation_variance  = {NAN, NAN, NAN};
      vo.velocity_variance     = {NAN, NAN, NAN};
    }

    vo.reset_counter = 0;
    vo.quality = 100;  // 0..100 (set sesuai health VIO kalau ada metrik)

    pub_->publish(vo);
  }

  // Members
  std::string in_odom_topic_, out_topic_;
  bool use_variance_{false}, sample_from_msg_stamp_{true};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<VehicleOdometry>::SharedPtr pub_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedVioToPx4>());
  rclcpp::shutdown();
  return 0;
}
