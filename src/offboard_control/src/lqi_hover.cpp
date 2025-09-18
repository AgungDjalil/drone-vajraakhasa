#include <chrono>
#include <cmath>
#include <numeric>
#include <vector>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class LqiHover : public rclcpp::Node
{
public:
  LqiHover(const std::string &name) : Node(name)
  {
    // ===== Parameters (tune di runtime via ros2 param set) =====
    // Target altitude (m, positif ke ATAS)
    target_alt_m_ = declare_parameter<float>("target_alt_m", 10.0f);

    // Gains LQI (diagonal)
    kpx_ = declare_parameter<double>("kpx", 1.2);
    kpy_ = declare_parameter<double>("kpy", 1.2);
    kpz_ = declare_parameter<double>("kpz", 1.8);

    kvx_ = declare_parameter<double>("kvx", 0.7);
    kvy_ = declare_parameter<double>("kvy", 0.7);
    kvz_ = declare_parameter<double>("kvz", 1.0);

    kix_ = declare_parameter<double>("kix", 0.10);
    kiy_ = declare_parameter<double>("kiy", 0.10);
    kiz_ = declare_parameter<double>("kiz", 0.20);

    // Limits
    v_max_xy_   = declare_parameter<double>("v_max_xy", 2.0);   // m/s (norm XY)
    vz_up_max_  = declare_parameter<double>("vz_up_max", -1.0); // m/s (NEGATIVE = UP)
    vz_dn_max_  = declare_parameter<double>("vz_dn_max",  1.0); // m/s (POSITIVE = DOWN)
    i_max_xy_   = declare_parameter<double>("i_max_xy",  1.0);  // anti-windup integral cap
    i_max_z_    = declare_parameter<double>("i_max_z",   1.5);

    // Phase durations
    settle_ticks_ = declare_parameter<int>("settle_ticks", 50);    // ~1s @50Hz
    land_after_ticks_ = declare_parameter<int>("land_after_ticks", 3000); // ~60s @50Hz

    // ===== PX4 Interfaces =====
    vehicle_local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1",
      rclcpp::SensorDataQoS(),
      std::bind(&LqiHover::localPositionCb, this, _1));

    offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    traj_pub_          = create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_cmd_pub_   = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

    // Timer 50 Hz
    timer_ = create_wall_timer(20ms, std::bind(&LqiHover::onTimer, this));
  }

private:
  enum class Phase { INIT, ARMED_SETTLE, TAKEOFF, HOLD };

  // --- Subscribers / Publishers ---
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --- State / measurements ---
  float curr_x_{0}, curr_y_{0}, curr_z_{0};
  float curr_vx_{0}, curr_vy_{0}, curr_vz_{0};
  float initial_x_{0}, initial_y_{0}, initial_z_{0}, initial_yaw_{0};
  bool initial_position_set_{false};

  // Averaging buffer for initial pose
  std::vector<double> xs_, ys_, zs_;

  // --- Control integrators ---
  double ex_i_{0}, ey_i_{0}, ez_i_{0};

  // --- Params / gains ---
  float target_alt_m_{1.0f};
  double kpx_, kpy_, kpz_;
  double kvx_, kvy_, kvz_;
  double kix_, kiy_, kiz_;
  double v_max_xy_, vz_up_max_, vz_dn_max_;
  double i_max_xy_, i_max_z_;
  int settle_ticks_, land_after_ticks_;

  // --- Phase / bookkeeping ---
  Phase phase_{Phase::INIT};
  int ticks_{0};

  // --- Helpers ---
  void localPositionCb(px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr msg)
  {
    curr_x_ = msg->x;
    curr_y_ = msg->y;
    curr_z_ = msg->z;
    curr_vx_ = msg->vx;
    curr_vy_ = msg->vy;
    curr_vz_ = msg->vz;

    if (!initial_position_set_) {
      xs_.push_back(msg->x);
      ys_.push_back(msg->y);
      zs_.push_back(msg->z);
      if (xs_.size() >= 20) {
        auto avg = [](const auto &v) {
          return std::accumulate(v.begin(), v.end(), 0.0) / std::max<size_t>(1, v.size());
        };
        initial_x_ = avg(xs_);
        initial_y_ = avg(ys_);
        initial_z_ = avg(zs_);
        initial_yaw_ = static_cast<float>(msg->heading);
        initial_position_set_ = true;
        phase_ = Phase::ARMED_SETTLE;

        RCLCPP_INFO(get_logger(),
          "Initial avg set: x=%.2f y=%.2f z=%.2f head=%.2f",
          initial_x_, initial_y_, initial_z_, initial_yaw_);
      }
    }

    // (Opsional) Print status singkat
    RCLCPP_DEBUG(get_logger(),
      "pos: (%.2f, %.2f, %.2f) vel: (%.2f, %.2f, %.2f)",
      curr_x_, curr_y_, curr_z_, curr_vx_, curr_vy_, curr_vz_);
  }

  void onTimer()
  {
    // Start offboard + arm setelah beberapa tick supaya stream setpoint sudah jalan
    if (ticks_ == 10) {
      // Set mode OFFBOARD (custom mode: main=PX4_CUSTOM_MAIN_MODE_OFFBOARD)
      publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
      arm();
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Sent OFFBOARD+ARM");
    }

    if (ticks_ >= land_after_ticks_) {
      land();
      timer_->cancel();
      return;
    }

    publishOffboardMode();
    publishLqiVelocitySetpoint();
    ticks_++;
  }

  void publishOffboardMode()
  {
    px4_msgs::msg::OffboardControlMode msg{};
    // Kita kirim velocity-only SP
    msg.position = false;
    msg.velocity = true;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = now_us();
    offboard_mode_pub_->publish(msg);
  }

  void publishLqiVelocitySetpoint()
  {
    if (!initial_position_set_) {
      // Belum punya referensi awal → kirim SP nol supaya tidak fail-safe
      px4_msgs::msg::TrajectorySetpoint sp{};
      const float NaN = std::numeric_limits<float>::quiet_NaN();
      sp.position = {NaN, NaN, NaN};
      sp.velocity = {0.f, 0.f, 0.f};
      sp.yaw = NaN;
      sp.timestamp = now_us();
      traj_pub_->publish(sp);
      return;
    }

    // ---- Tentukan referensi posisi (x*, y*, z*) per phase ----
    float x_ref = initial_x_;
    float y_ref = initial_y_;
    float z_ref = initial_z_; // NED positive down

    if (phase_ == Phase::ARMED_SETTLE) {
      // Diam di titik awal selama settle_ticks
      if (ticks_ > 10 + settle_ticks_) {
        phase_ = Phase::TAKEOFF;
      }
    } else if (phase_ == Phase::TAKEOFF) {
      // Target ketinggian: initial_z - target_alt (ingat: naik = z lebih NEGATIF)
      z_ref = initial_z_ - target_alt_m_;
      // Saat sudah naik cukup dekat → HOLD
      const float climbed = initial_z_ - curr_z_; // positif kalau sudah naik
      if (climbed >= (target_alt_m_ - 0.05f)) {
        phase_ = Phase::HOLD;
      }
    } else if (phase_ == Phase::HOLD) {
      z_ref = initial_z_ - target_alt_m_;
    }

    // ---- LQI per-axis ----
    // Error (posisi)
    double ex = static_cast<double>(curr_x_ - x_ref);
    double ey = static_cast<double>(curr_y_ - y_ref);
    double ez = static_cast<double>(curr_z_ - z_ref);

    // dt ~ 0.02 s (50 Hz)
    constexpr double dt = 0.02;

    // Integrator (anti-windup clamp)
    ex_i_ = clamp(ex_i_ + ex * dt, -i_max_xy_, i_max_xy_);
    ey_i_ = clamp(ey_i_ + ey * dt, -i_max_xy_, i_max_xy_);
    ez_i_ = clamp(ez_i_ + ez * dt, -i_max_z_,  i_max_z_);

    // Velocity feedback (derivative via measured vel)
    double vx_meas = static_cast<double>(curr_vx_);
    double vy_meas = static_cast<double>(curr_vy_);
    double vz_meas = static_cast<double>(curr_vz_);

    // Kontrol LQI (v_cmd)
    double vx_cmd = -(kpx_ * ex + kvx_ * vx_meas + kix_ * ex_i_);
    double vy_cmd = -(kpy_ * ey + kvy_ * vy_meas + kiy_ * ey_i_);
    double vz_cmd = -(kpz_ * ez + kvz_ * vz_meas + kiz_ * ez_i_);
    // Ingat: kalau ez > 0 (drone lebih "bawah" dari target), maka
    // kpz*ez positif → vz_cmd negatif → NAiK (sesuai NED). Good.

    // ---- Limit kecepatan ----
    // XY norm limit
    const double vxy_norm = std::hypot(vx_cmd, vy_cmd);
    if (vxy_norm > v_max_xy_) {
      const double s = v_max_xy_ / std::max(1e-6, vxy_norm);
      vx_cmd *= s;
      vy_cmd *= s;
    }
    // Z limit (ingat NED: up negative, down positive)
    if (vz_cmd < vz_up_max_) vz_cmd = vz_up_max_;   // contoh: -1.0 m/s limit naik
    if (vz_cmd > vz_dn_max_) vz_cmd = vz_dn_max_;   // contoh: +1.0 m/s limit turun

    // ---- Publish TrajectorySetpoint (velocity-only) ----
    px4_msgs::msg::TrajectorySetpoint sp{};
    const float NaN = std::numeric_limits<float>::quiet_NaN();
    sp.position = {NaN, NaN, NaN};
    sp.velocity = {
      static_cast<float>(vx_cmd),
      static_cast<float>(vy_cmd),
      static_cast<float>(vz_cmd)
    };
    sp.yaw = initial_yaw_; // hold heading awal
    sp.timestamp = now_us();
    traj_pub_->publish(sp);

    // Debug ringkas tiap 0.5s
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "[%s] e=(%.2f,%.2f,%.2f) vcmd=(%.2f,%.2f,%.2f) zref=%.2f",
      phaseName().c_str(), ex, ey, ez, sp.velocity[0], sp.velocity[1], sp.velocity[2], z_ref);
  }

  void arm()
  {
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(get_logger(), "Arm command sent");
  }

  void land()
  {
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
    RCLCPP_INFO(get_logger(), "Land command sent");
  }

  void publishVehicleCommand(uint16_t command, float p1 = 0.f, float p2 = 0.f)
  {
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = p1;
    msg.param2 = p2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = now_us();
    vehicle_cmd_pub_->publish(msg);
  }

  inline uint64_t now_us()
  {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  }

  static inline double clamp(double v, double lo, double hi)
  {
    return std::min(std::max(v, lo), hi);
  }

  std::string phaseName() const
  {
    switch (phase_) {
      case Phase::INIT: return "INIT";
      case Phase::ARMED_SETTLE: return "SETTLE";
      case Phase::TAKEOFF: return "TAKEOFF";
      case Phase::HOLD: return "HOLD";
    }
    return "UNKNOWN";
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LqiHover>("lqi_hover");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
