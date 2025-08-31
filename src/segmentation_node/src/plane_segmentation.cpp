#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/ModelCoefficients.h>

#include <Eigen/Dense>
#include <numeric>
#include <vector>
#include <deque>     // <--- perlu untuk BFS
#include <cmath>

class ZedSegmentation : public rclcpp::Node {
  public:
    ZedSegmentation() : Node("plane_segmentation") {
      // I/O topics
      this->declare_parameter<std::string>("input_topic", "/zed/zed_node/point_cloud/cloud_registered");
      this->declare_parameter<std::string>("output_topic", "/segmented");

      // Preprocess
      this->declare_parameter<double>("z_min", -5.0);
      this->declare_parameter<double>("z_max",  5.0);
      this->declare_parameter<double>("leaf_size", 0.05);

      this->declare_parameter<bool>("use_sor", true);
      this->declare_parameter<int>("sor_mean_k", 30);
      this->declare_parameter<double>("sor_std_mul", 1.0);
      this->declare_parameter<double>("ror_radius", 0.25);
      this->declare_parameter<int>("ror_min_neighbors", 8);

      // Plane
      this->declare_parameter<int>("min_plane_inliers", 500);
      this->declare_parameter<double>("plane_distance_thresh", 0.17);
      this->declare_parameter<int>("plane_max_iterations", 500);

      // Zona pewarnaan (meter)
      this->declare_parameter<double>("outer_diameter", 7.0);   // area oranye
      this->declare_parameter<double>("safe_diameter",  2.0);   // patch hijau (disk)

      // Grid & kriteria
      this->declare_parameter<double>("grid_cell",       0.25); // m
      this->declare_parameter<double>("safe_fill_ratio", 0.90); // 70%
      this->declare_parameter<int>("min_pts_per_cell",   5);    // titik/sel
      this->declare_parameter<double>("normal_eps",      0.05); // kerataan (m), 2 cm

      const auto in  = this->get_parameter("input_topic").as_string();
      const auto out = this->get_parameter("output_topic").as_string();

      sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        in, rclcpp::SensorDataQoS(),
        std::bind(&ZedSegmentation::cloudCb, this, std::placeholders::_1));

      pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(out, 10);

      RCLCPP_INFO(get_logger(), "segmented node: %s -> %s", in.c_str(), out.c_str());
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    void cloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      // Convert input
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(*msg, *cloud);
      if (cloud->empty()) return;

      // Params
      const double z_min   = this->get_parameter("z_min").as_double();
      const double z_max   = this->get_parameter("z_max").as_double();
      const float  leaf    = static_cast<float>(this->get_parameter("leaf_size").as_double());
      const bool   use_sor = this->get_parameter("use_sor").as_bool();
      const int    min_plane_inliers = this->get_parameter("min_plane_inliers").as_int();
      const double dist_thresh       = this->get_parameter("plane_distance_thresh").as_double();
      const int    max_iters         = this->get_parameter("plane_max_iterations").as_int();

      const float  r_outer = 0.5f * static_cast<float>(this->get_parameter("outer_diameter").as_double());
      const float  r_safe  = 0.5f * static_cast<float>(this->get_parameter("safe_diameter").as_double());
      const float  cell    = static_cast<float>(this->get_parameter("grid_cell").as_double());
      const float  required_ratio = static_cast<float>(this->get_parameter("safe_fill_ratio").as_double());
      const int    min_pts_per_cell = this->get_parameter("min_pts_per_cell").as_int();
      const float  normal_eps = static_cast<float>(this->get_parameter("normal_eps").as_double());

      // ROI
      pcl::PointCloud<pcl::PointXYZ>::Ptr roi(new pcl::PointCloud<pcl::PointXYZ>);
      {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(z_min, z_max);
        pass.filter(*roi);
      }
      if (roi->empty()) return;

      // Downsample
      pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>);
      {
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(roi);
        vg.setLeafSize(leaf, leaf, leaf);
        vg.filter(*down);
      }
      if (down->empty()) return;

      // Denoise
      pcl::PointCloud<pcl::PointXYZ>::Ptr denoised(new pcl::PointCloud<pcl::PointXYZ>);
      if (use_sor) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(down);
        sor.setMeanK(this->get_parameter("sor_mean_k").as_int());
        sor.setStddevMulThresh(this->get_parameter("sor_std_mul").as_double());
        sor.filter(*denoised);
      } else {
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
        ror.setInputCloud(down);
        ror.setRadiusSearch(this->get_parameter("ror_radius").as_double());
        ror.setMinNeighborsInRadius(this->get_parameter("ror_min_neighbors").as_int());
        ror.filter(*denoised);
      }
      if (denoised->empty()) return;

      // Segment plane
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(dist_thresh);
      seg.setMaxIterations(max_iters);

      pcl::ModelCoefficients coeffs;
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      seg.setInputCloud(denoised);
      seg.segment(*inliers, coeffs);

      if ((int)inliers->indices.size() < min_plane_inliers) {
        RCLCPP_WARN(get_logger(), "No dominant plane found");
        return;
      }

      // Extract plane points
      pcl::PointCloud<pcl::PointXYZ>::Ptr plane_only(new pcl::PointCloud<pcl::PointXYZ>);
      {
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(denoised);
        extract.setIndices(inliers);
        extract.filter(*plane_only);
      }
      if (plane_only->empty()) return;

      // Centroid
      Eigen::Vector4f c4;
      pcl::compute3DCentroid(*plane_only, c4);
      const float cx = c4[0], cy = c4[1], cz = c4[2];

      // Normal (unit) & basis (t1,t2)
      Eigen::Vector3f n(coeffs.values[0], coeffs.values[1], coeffs.values[2]);
      if (n.norm() == 0.f) n = Eigen::Vector3f(0,0,1);
      n.normalize();
      Eigen::Vector3f a = (std::abs(n.z()) < 0.9f) ? Eigen::Vector3f(0,0,1) : Eigen::Vector3f(0,1,0);
      Eigen::Vector3f t1 = (a - a.dot(n)*n).normalized();
      Eigen::Vector3f t2 = n.cross(t1);

      const float t1x = t1.x(), t1y = t1.y(), t1z = t1.z();
      const float t2x = t2.x(), t2y = t2.y(), t2z = t2.z();
      const float r2_outer = r_outer * r_outer;

      // ============================
      // 1) Grid metrik (N x N)
      // ============================
      const int N = std::max(1, (int)std::ceil((2.f * r_outer) / cell));
      auto IDX = [N](int ix, int iy){ return iy * N + ix; };

      std::vector<int>   counts(N*N, 0);
      std::vector<float> sum_r(N*N, 0.f), sum_r2(N*N, 0.f); // <--- tambahkan untuk kerataan

      for (const auto& p : plane_only->points) {
        const float dx = p.x - cx, dy = p.y - cy, dz = p.z - cz;
        const float u  = dx*t1x + dy*t1y + dz*t1z;
        const float v  = dx*t2x + dy*t2y + dz*t2z;
        const float d2 = u*u + v*v;
        if (d2 <= r2_outer) {
          int ix = (int)std::floor((u + r_outer)/cell);
          int iy = (int)std::floor((v + r_outer)/cell);
          if (ix>=0 && ix<N && iy>=0 && iy<N) {
            counts[IDX(ix,iy)]++;
            // residu terhadap plane (jarak normal)
            const float r = dx*n.x() + dy*n.y() + dz*n.z();
            sum_r [IDX(ix,iy)] += r;
            sum_r2[IDX(ix,iy)] += r*r;
          }
        }
      }

      // Offsets disk safe (satuan sel)
      const int R_safe_cells  = std::max(1, (int)std::round(r_safe  / cell));
      const int R_outer_cells = std::max(1, (int)std::round(r_outer / cell));
      std::vector<std::pair<int,int>> disk_offsets_safe;
      disk_offsets_safe.reserve((2*R_safe_cells+1)*(2*R_safe_cells+1));
      for (int dyc = -R_safe_cells; dyc <= R_safe_cells; ++dyc) {
        for (int dxc = -R_safe_cells; dxc <= R_safe_cells; ++dxc) {
          if (dxc*dxc + dyc*dyc <= R_safe_cells*R_safe_cells)
            disk_offsets_safe.emplace_back(dxc, dyc);
        }
      }
      const int disk_area_safe = (int)disk_offsets_safe.size();

      // ===============================
      // 2) Cari semua pusat safe center (cakupan)
      // ===============================
      std::vector<uint8_t> safe_center(N*N, 0);
      for (int iy = 0; iy < N; ++iy) {
        for (int ix = 0; ix < N; ++ix) {
          const int dx_c = ix - N/2, dy_c = iy - N/2;
          if (dx_c*dx_c + dy_c*dy_c > R_outer_cells*R_outer_cells) continue;
          if (ix < R_safe_cells || ix >= N - R_safe_cells ||
              iy < R_safe_cells || iy >= N - R_safe_cells) continue;

          int filled_cells = 0;
          for (const auto& o : disk_offsets_safe) {
            const int jx = ix + o.first, jy = iy + o.second;
            if (counts[IDX(jx,jy)] >= min_pts_per_cell) filled_cells++;
          }
          const float ratio = (float)filled_cells / (float)disk_area_safe;
          if (ratio >= required_ratio) safe_center[IDX(ix,iy)] = 1;
        }
      }

      // ----------------------------------------------------
      // 3) Pilih 1 pusat: max clearance ke obstacle,
      //    tie-break: paling dekat ke pusat; buat safe_mask
      // ----------------------------------------------------
      auto in_bounds = [N](int x,int y){ return (x>=0 && x<N && y>=0 && y<N); };

      // cell_ok + obstacle (pakai kerataan)
      std::vector<uint8_t> obstacle(N*N, 0);
      for (int iy = 0; iy < N; ++iy) {
        for (int ix = 0; ix < N; ++ix) {
          const int id = IDX(ix,iy);
          bool ok = false;
          if (counts[id] >= min_pts_per_cell) {
            const float c   = (float)counts[id];
            const float mean= sum_r[id]/c;
            const float var = std::max(0.f, sum_r2[id]/c - mean*mean);
            const float st  = std::sqrt(var);
            ok = (std::abs(mean) <= normal_eps && st <= normal_eps);
          }
          const int dx_c = ix - N/2, dy_c = iy - N/2;
          const bool inside_orange = (dx_c*dx_c + dy_c*dy_c <= R_outer_cells*R_outer_cells);
          obstacle[id] = (inside_orange && !ok) ? 1 : 0;
        }
      }

      // Distance transform (BFS 8-neigh)
      const int INF = 1e9;
      std::vector<int> dist(N*N, INF);
      std::deque<std::pair<int,int>> q;
      for (int iy = 0; iy < N; ++iy)
        for (int ix = 0; ix < N; ++ix)
          if (obstacle[IDX(ix,iy)]) { dist[IDX(ix,iy)] = 0; q.emplace_back(ix,iy); }

      if (q.empty()) std::fill(dist.begin(), dist.end(), R_outer_cells*2);

      const int dx8[8]={-1,0,1,-1,1,-1,0,1};
      const int dy8[8]={-1,-1,-1,0,0,1,1,1};
      while(!q.empty()){
        auto [x,y]=q.front(); q.pop_front();
        int dcur = dist[IDX(x,y)];
        for(int k=0;k<8;++k){
          int nx=x+dx8[k], ny=y+dy8[k];
          if(!in_bounds(nx,ny)) continue;
          if(dist[IDX(nx,ny)] > dcur + 1){
            dist[IDX(nx,ny)] = dcur + 1;
            q.emplace_back(nx,ny);
          }
        }
      }

      // pilih pusat terbaik
      int best_ix = -1, best_iy = -1;
      float best_clearance = -1e9f, best_center_d2 = 1e30f;
      for (int iy = 0; iy < N; ++iy) {
        for (int ix = 0; ix < N; ++ix) {
          if (!safe_center[IDX(ix,iy)]) continue;
          float clearance_m = dist[IDX(ix,iy)] * cell - r_safe;
          float dcx = (ix - N/2) * cell, dcy = (iy - N/2) * cell;
          float center_d2 = dcx*dcx + dcy*dcy;
          if (clearance_m > best_clearance + 1e-6f ||
              (std::abs(clearance_m - best_clearance) <= 1e-6f && center_d2 < best_center_d2)) {
            best_clearance = clearance_m;
            best_center_d2 = center_d2;
            best_ix = ix; best_iy = iy;
          }
        }
      }

      // safe_mask = 1 lingkaran di pusat terpilih
      std::vector<uint8_t> safe_mask(N*N, 0);
      if (best_ix >= 0) {
        for (const auto& o : disk_offsets_safe) {
          const int jx = best_ix + o.first, jy = best_iy + o.second;
          if (in_bounds(jx,jy)) safe_mask[IDX(jx,jy)] = 1;
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000,
          "Chosen safe center (%d,%d): clearance=%.2fm, center_dist=%.2fm",
          best_ix, best_iy, std::max(0.f,best_clearance), std::sqrt(best_center_d2));
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
          "No safe center found in orange zone.");
      }

      // ===========================
      // 4) Pewarnaan point cloud RGB
      // ===========================
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>);
      colored->points.resize(plane_only->points.size());
      colored->width  = (uint32_t)plane_only->points.size();
      colored->height = 1;
      colored->is_dense = plane_only->is_dense;

      const uint8_t GR_R=0,   GR_G=255, GR_B=0;     // Hijau (safe)
      const uint8_t OR_R=255, OR_G=165, OR_B=0;     // Oranye (kandidat)
      const uint8_t WH_R=255, WH_G=255, WH_B=255;   // Putih (di luar)

      #pragma omp parallel for schedule(static)
      for (long i = 0; i < (long)plane_only->points.size(); ++i) {
        const auto& p = plane_only->points[(size_t)i];
        const float dx = p.x - cx, dy = p.y - cy, dz = p.z - cz;
        const float u  = dx*t1x + dy*t1y + dz*t1z;
        const float v  = dx*t2x + dy*t2y + dz*t2z;

        auto& q = colored->points[(size_t)i];
        q.x = p.x; q.y = p.y; q.z = p.z;

        const float d2_outer = u*u + v*v;
        if (d2_outer > r2_outer) { q.r = WH_R; q.g = WH_G; q.b = WH_B; }
        else {
          int ix = (int)std::floor((u + r_outer)/cell);
          int iy = (int)std::floor((v + r_outer)/cell);
          bool in_safe = (ix>=0 && ix<N && iy>=0 && iy<N) ? (safe_mask[IDX(ix,iy)] != 0) : false;
          if (in_safe) { q.r = GR_R; q.g = GR_G; q.b = GR_B; }
          else         { q.r = OR_R; q.g = OR_G; q.b = OR_B; }
        }
      }

      // Publish
      sensor_msgs::msg::PointCloud2 out;
      pcl::toROSMsg(*colored, out);
      out.header = msg->header;
      pub_->publish(out);
    }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedSegmentation>());
  rclcpp::shutdown();
  return 0;
}
