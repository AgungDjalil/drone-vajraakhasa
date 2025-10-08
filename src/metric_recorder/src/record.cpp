// file: src/data_recorder.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <unordered_map>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <limits>
#include <cmath>

// TF
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/time.h>

using std::placeholders::_1;

struct Triple {
  // sizes
  int n_in = -1;        // ZED raw
  int n_cropped = -1;   // after crop
  int n_plane = -1;     // /segmented

  // times (steady_clock)
  std::chrono::steady_clock::time_point t_circle;   // after crop (anchor)
  std::chrono::steady_clock::time_point t_segment;  // when /segmented arrives

  // TF timing measured when /plane_colored arrives
  bool   has_tf_time = false;
  double T_tf_ms = std::numeric_limits<double>::quiet_NaN();

  // safety point from TF (in target_frame)
  double sx = NAN, sy = NAN, sz = NAN;              // z disimpan, tak ditulis

  // write guard
  bool wrote = false;
};

class DataRecorder : public rclcpp::Node {
public:
  DataRecorder() : Node("data_recorder")
  {
    // Params
    algo_          = declare_parameter<std::string>("algo", "GNG"); // atau "RANSAC"
    csv_path_      = declare_parameter<std::string>("csv_path", "algo_metrics.csv");
    topic_raw_     = declare_parameter<std::string>("topic_raw", "/zed/zed_node/point_cloud/cloud_registered");
    topic_circle_  = declare_parameter<std::string>("topic_circle", "/circle_cloud");
    topic_segment_ = declare_parameter<std::string>("topic_segment", "/segment");
    topic_colored_ = declare_parameter<std::string>("topic_colored", "/plane_colored"); // NEW

    // TF params
    target_frame_  = declare_parameter<std::string>("target_frame", "odom");
    safety_frame_  = declare_parameter<std::string>("safety_frame", "safety_point");
    tf_timeout_s_  = declare_parameter<double>("tf_timeout_s", 0.1); // 100 ms

    // TF objects
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // CSV
    file_.open(csv_path_, std::ios::out);
    if (!file_) {
      RCLCPP_FATAL(get_logger(), "Failed to open CSV at %s", csv_path_.c_str());
    }
    file_ << "stamp_ns,algo,n_in,n_cropped,n_plane,"
             "T_seg_ms,T_tf_ms,"
             "safe_x,safe_y\n";

    // Subs
    sub_raw_     = create_subscription<sensor_msgs::msg::PointCloud2>(topic_raw_, rclcpp::SensorDataQoS(),
                      std::bind(&DataRecorder::cb_raw, this, _1));
    sub_circle_  = create_subscription<sensor_msgs::msg::PointCloud2>(topic_circle_, rclcpp::SensorDataQoS(),
                      std::bind(&DataRecorder::cb_circle, this, _1));
    sub_segment_ = create_subscription<sensor_msgs::msg::PointCloud2>(topic_segment_, rclcpp::SensorDataQoS(),
                      std::bind(&DataRecorder::cb_segment, this, _1));
    sub_colored_ = create_subscription<sensor_msgs::msg::PointCloud2>(topic_colored_, rclcpp::SensorDataQoS(),
                      std::bind(&DataRecorder::cb_colored, this, _1)); // NEW

    // Housekeeping: bersihkan entri lama (mis. 10 detik sejak event terakhir)
    gc_timer_ = create_wall_timer(std::chrono::seconds(5), std::bind(&DataRecorder::gc, this));

    RCLCPP_INFO(get_logger(),
      "DataRecorder ready → CSV: %s (algo=%s)\n"
      "  WRITE on /plane_colored (needs /circle_cloud + /segmented present)\n"
      "  TF timing measured on %s | TF: %s ← %s (timeout=%.0f ms)",
      csv_path_.c_str(), algo_.c_str(), topic_colored_.c_str(),
      target_frame_.c_str(), safety_frame_.c_str(), tf_timeout_s_*1000.0);
  }

private:
  // Helpers
  static inline uint64_t stamp_key(const std_msgs::msg::Header& h) {
    return (static_cast<uint64_t>(h.stamp.sec) * 1000000000ULL) + static_cast<uint64_t>(h.stamp.nanosec);
  }
  static inline int pc_size(const sensor_msgs::msg::PointCloud2& m) {
    return static_cast<int>(m.width * m.height);
  }

  bool try_get_safety_point(const rclcpp::Time& stamp, double& x, double& y, double& z)
  {
    try {
      // case A: lookup by stamp (pakai rclcpp::Time + rclcpp::Duration)
      auto tf = tf_buffer_->lookupTransform(
        target_frame_, safety_frame_, stamp,
        rclcpp::Duration::from_seconds(tf_timeout_s_));
      x = tf.transform.translation.x;
      y = tf.transform.translation.y;
      z = tf.transform.translation.z;
      return true;
    } catch (const std::exception& /*e1*/) {
      try {
        // case B: fallback latest (pakai tf2::TimePointZero + tf2::Duration)
        auto tf = tf_buffer_->lookupTransform(
          target_frame_, safety_frame_, tf2::TimePointZero,
          tf2::durationFromSec(tf_timeout_s_));
        x = tf.transform.translation.x;
        y = tf.transform.translation.y;
        z = tf.transform.translation.z;
        return true;
      } catch (const std::exception& e2) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "TF %s <- %s not available (stamp or latest): %s",
          target_frame_.c_str(), safety_frame_.c_str(), e2.what());
        return false;
      }
    }
  }

  static inline double ms_between(const std::chrono::steady_clock::time_point& a,
                                  const std::chrono::steady_clock::time_point& b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
  }

  // Try write when all pieces are ready and not yet written
  void maybe_write(const uint64_t k, Triple& t)
  {
    if (t.wrote) return;

    const bool have_anchor  = (t.t_circle.time_since_epoch().count()  != 0);
    const bool have_segment = (t.t_segment.time_since_epoch().count() != 0);
    const bool have_tf      = t.has_tf_time;

    if (!(have_anchor && have_segment && have_tf)) return;

    const double T_seg_ms = ms_between(t.t_circle, t.t_segment);

    file_ << k << "," << algo_ << ","
          << (t.n_in>=0 ? t.n_in : 0) << ","
          << (t.n_cropped>=0 ? t.n_cropped : 0) << ","
          << (t.n_plane>=0 ? t.n_plane : 0) << ","
          << std::fixed << std::setprecision(3)
          << T_seg_ms << ","
          << (std::isfinite(t.T_tf_ms) ? t.T_tf_ms : 0.0) << ",";

    if (std::isfinite(t.sx) && std::isfinite(t.sy)) {
      file_ << t.sx << "," << t.sy << "\n";
    } else {
      file_ << "," << "," << "\n";  // safe_x,safe_y kosong → ",,"
    }

    t.wrote = true;
  }

  // Callbacks
  void cb_raw(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const uint64_t k = stamp_key(msg->header);
    auto &t = table_[k];
    t.n_in = pc_size(*msg);
  }

  void cb_circle(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const uint64_t k = stamp_key(msg->header);
    auto &t = table_[k];
    t.n_cropped = pc_size(*msg);
    t.t_circle = std::chrono::steady_clock::now();   // anchor for T_seg
    maybe_write(k, t);
  }

  void cb_segment(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const uint64_t k = stamp_key(msg->header);
    auto &t = table_[k];
    t.n_plane = pc_size(*msg);
    t.t_segment = std::chrono::steady_clock::now();
    // Tidak menulis CSV di sini; biarkan ditulis saat /plane_colored (setelah ukur T_tf_ms)
    maybe_write(k, t);
  }

  // NEW: ukur lama TF lookup saat /plane_colored datang, lalu tulis CSV
  void cb_colored(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const uint64_t k = stamp_key(msg->header);
    auto &t = table_[k];

    // Ukur durasi TF lookup
    auto t0 = std::chrono::steady_clock::now();
    double sx, sy, sz;
    bool got_tf = try_get_safety_point(rclcpp::Time(msg->header.stamp), sx, sy, sz);
    auto t1 = std::chrono::steady_clock::now();

    t.T_tf_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    t.has_tf_time = true;
    if (got_tf) { t.sx = sx; t.sy = sy; t.sz = sz; }

    maybe_write(k, t);
  }

  void gc()
  {
    // Buang entri lebih tua dari ~10 detik sejak event terakhir
    const auto now = std::chrono::steady_clock::now();
    for (auto it = table_.begin(); it != table_.end(); ) {
      double age_s = 0.0;
      const auto &rec = it->second;

      if (rec.t_segment.time_since_epoch().count() != 0) {
        age_s = std::chrono::duration<double>(now - rec.t_segment).count();
      } else if (rec.t_circle.time_since_epoch().count() != 0) {
        age_s = std::chrono::duration<double>(now - rec.t_circle).count();
      }

      if (rec.wrote || age_s > 10.0) it = table_.erase(it);
      else ++it;
    }
  }

  // Params/state
  std::string algo_, csv_path_;
  std::string topic_raw_, topic_circle_, topic_segment_, topic_colored_;
  std::string target_frame_, safety_frame_;
  double tf_timeout_s_;

  std::ofstream file_;
  std::unordered_map<uint64_t, Triple> table_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Subs & timer
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_raw_, sub_circle_, sub_segment_, sub_colored_;
  rclcpp::TimerBase::SharedPtr gc_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataRecorder>());
  rclcpp::shutdown();
  return 0;
}
