#include <chrono>
#include <limits>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

using namespace std::chrono_literals;
using px4_msgs::msg::VehicleCommand;

class GearServiceNode : public rclcpp::Node {
public:
  GearServiceNode() : rclcpp::Node("gear_service_node")
  {
    pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", rclcpp::QoS(10));

    sysid_           = declare_parameter<int>("target_system", 1);
    compid_          = declare_parameter<int>("target_component", 1);
    src_sys_         = declare_parameter<int>("source_system", 255);
    src_comp_        = declare_parameter<int>("source_component", 190);

    value_up_        = declare_parameter<double>("value_up",   1.0);   // +1.0 => UP (retract)
    value_down_      = declare_parameter<double>("value_down", -1.0);  // -1.0 => DOWN (deploy)

    repeats_up_      = declare_parameter<int>("repeats_up",    5);
    repeats_down_    = declare_parameter<int>("repeats_down",  3);
    interval_ms_     = declare_parameter<int>("interval_ms",   150);

    wait_ms_         = declare_parameter<int>("wait_ms",         2000); // wait for subscriber discovery
    start_delay_ms_  = declare_parameter<int>("start_delay_ms",   300); // pause after match
    hold_up_ms_      = declare_parameter<int>("hold_up_ms",      5000); // hold after UP

    // Services
    srv_run_seq_ = create_service<std_srvs::srv::Trigger>(
      "gear/run_sequence",
      std::bind(&GearServiceNode::onRunSequence, this, std::placeholders::_1, std::placeholders::_2));

    srv_set_ud_ = create_service<std_srvs::srv::SetBool>(
      "gear/set_up_down",
      std::bind(&GearServiceNode::onSetUpDown, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(),
      "Gear service ready. Services:\n  - /gear/run_sequence (Trigger)\n  - /gear/set_up_down (SetBool: true=UP, false=DOWN)");
  }

private:
  // ---- Service callbacks -----------------------------------------------------

  void onRunSequence(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    RCLCPP_INFO(get_logger(),
      "Requested: RUN SEQUENCE  [UP x%d -> hold %d ms -> DOWN x%d] (interval %d ms)",
      repeats_up_, hold_up_ms_, repeats_down_, interval_ms_);

    if (!waitForSubscriber()) {
      RCLCPP_WARN(get_logger(), "No subscriber matched. Proceeding anyway.");
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(start_delay_ms_));
    }

    // Send UP
    for (int i = 0; i < repeats_up_; ++i) {
      send_param1(static_cast<float>(value_up_));
      if (i + 1 < repeats_up_) std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms_));
    }

    if (hold_up_ms_ > 0) {
      RCLCPP_INFO(get_logger(), "Hold after UP for %.1f s ...", hold_up_ms_ / 1000.0);
      std::this_thread::sleep_for(std::chrono::milliseconds(hold_up_ms_));
    }

    // Send DOWN
    for (int i = 0; i < repeats_down_; ++i) {
      send_param1(static_cast<float>(value_down_));
      if (i + 1 < repeats_down_) std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms_));
    }

    res->success = true;
    res->message = "Sequence completed: UP -> hold -> DOWN";
  }

  void onSetUpDown(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    const bool up = req->data;
    const float value = up ? static_cast<float>(value_up_) : static_cast<float>(value_down_);
    const int repeats = up ? repeats_up_ : repeats_down_;

    RCLCPP_INFO(get_logger(), "Requested: %s x%d (interval %d ms)",
                up ? "UP" : "DOWN", repeats, interval_ms_);

    if (!waitForSubscriber()) {
      RCLCPP_WARN(get_logger(), "No subscriber matched. Proceeding anyway.");
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(start_delay_ms_));
    }

    for (int i = 0; i < repeats; ++i) {
      send_param1(value);
      if (i + 1 < repeats) std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms_));
    }

    res->success = true;
    res->message = std::string("Sent ") + (up ? "UP" : "DOWN");
  }

  // ---- Helpers ---------------------------------------------------------------

  bool waitForSubscriber()
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(wait_ms_);
    while (pub_->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                           "Waiting for /fmu/in/vehicle_command subscriber...");
      std::this_thread::sleep_for(50ms);
    }
    if (pub_->get_subscription_count() > 0) {
      RCLCPP_INFO(get_logger(), "Matched %zu subscriber(s).", pub_->get_subscription_count());
      return true;
    }
    return false;
  }

  void send_param1(float value)
  {
    VehicleCommand cmd{};
    cmd.timestamp       = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000ULL); // us
    cmd.command         = VehicleCommand::VEHICLE_CMD_DO_SET_ACTUATOR; // 187
    float nan = std::numeric_limits<float>::quiet_NaN();
    cmd.param1 = value; // Actuator Set 1
    cmd.param2 = nan; cmd.param3 = nan; cmd.param4 = nan; cmd.param5 = nan; cmd.param6 = nan;

    cmd.target_system    = static_cast<uint8_t>(sysid_);
    cmd.target_component = static_cast<uint8_t>(compid_);
    cmd.source_system    = static_cast<uint8_t>(src_sys_);
    cmd.source_component = static_cast<uint8_t>(src_comp_);
    cmd.from_external    = true;

    pub_->publish(cmd);
    RCLCPP_INFO(get_logger(), "Sent DO_SET_ACTUATOR: param1=%.2f (%s)",
                value, (value >= 0.f ? "UP/retract" : "DOWN/deploy"));
  }

  // ---- Members ---------------------------------------------------------------

  rclcpp::Publisher<VehicleCommand>::SharedPtr pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr  srv_run_seq_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr  srv_set_ud_;

  int sysid_, compid_, src_sys_, src_comp_;
  double value_up_, value_down_;
  int repeats_up_, repeats_down_, interval_ms_;
  int wait_ms_, start_delay_ms_, hold_up_ms_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GearServiceNode>();

  // Use MultiThreadedExecutor if you expect to call long services concurrently
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
