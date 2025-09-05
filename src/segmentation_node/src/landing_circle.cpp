#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <limits>
#include <algorithm>
#include <vector>
#include <deque>
#include <cmath>
#include <string>

class SafeLandingColor : public rclcpp::Node {
public:
  SafeLandingColor() : Node("safe_landing_color") {
    // I/O
    input_topic_    = declare_parameter<std::string>("input_topic",   "/segmented");
    output_topic_   = declare_parameter<std::string>("output_topic",  "/plane_colored");

    // Proyeksi bidang
    plane_axes_     = declare_parameter<std::string>("plane_axes",    "yz"); // "xy","yz","xz"

    // Geometri area (HANYA diameter aman)
    safe_diameter_  = declare_parameter<double>("safe_diameter",      1.0);  // meter (disk hijau)

    // Grid & kriteria
    grid_cell_      = declare_parameter<double>("grid_cell",          0.25); // meter/sel
    min_pts_cell_   = declare_parameter<int>("min_pts_per_cell",      20);
    fill_ratio_req_ = declare_parameter<double>("safe_fill_ratio",    0.95); // lebih ketat
    close_gaps_     = declare_parameter<bool>("close_gaps",           true);
    inflate_cells_  = declare_parameter<int>("inflate_cells",         1);    // buffer obstacle (sel)

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SafeLandingColor::cbCloud, this, std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 1);

    RCLCPP_INFO(get_logger(),
      "SafeLandingColor(no-outer): axes=%s safe_d=%.2fm grid=%.2fm fill>=%.2f pts/sel>=%d inflate=%d",
      plane_axes_.c_str(), safe_diameter_, grid_cell_, fill_ratio_req_, min_pts_cell_, inflate_cells_);
  }

private:
  inline void pickUV(const pcl::PointXYZ &p, float &u, float &v) const {
    if (plane_axes_ == "xy")      { u = p.x; v = p.y; }
    else if (plane_axes_ == "yz") { u = p.y; v = p.z; }
    else /* xz */                 { u = p.x; v = p.z; }
  }

  void cbCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Input cloud empty");
      publishOnlySafe(*cloud, {}, false, 0.f, 0.f, 0.f, msg->header);
      return;
    }

    // 1) Proyeksi ke (u,v) dan bounding box
    float umin= std::numeric_limits<float>::max();
    float vmin= std::numeric_limits<float>::max();
    float umax=-std::numeric_limits<float>::max();
    float vmax=-std::numeric_limits<float>::max();

    std::vector<std::pair<float,float>> uv; uv.reserve(cloud->size());
    uv.resize(cloud->size());
    for (size_t i=0;i<cloud->size();++i) {
      float u,v; pickUV((*cloud)[i], u, v);
      uv[i] = {u,v};
      umin = std::min(umin,u); umax = std::max(umax,u);
      vmin = std::min(vmin,v); vmax = std::max(vmax,v);
    }

    // sedikit margin biar nggak mepet tepi grid
    const float margin = 0.25f; // 25 cm
    umin -= margin; umax += margin;
    vmin -= margin; vmax += margin;

    // 2) Parameter grid & radius aman
    const float r_safe  = (float)(0.5 * safe_diameter_);
    const float cell    = (float)grid_cell_;
    const int   cols    = std::max(1, (int)std::ceil((umax - umin)/cell));
    const int   rows    = std::max(1, (int)std::ceil((vmax - vmin)/cell));
    if ((long long)rows * (long long)cols > 10'000'000LL) {
      RCLCPP_WARN(get_logger(), "Grid too large (%dx%d). Naikkan grid_cell.", cols, rows);
      publishOnlySafe(*cloud, uv, false, 0.f, 0.f, 0.f, msg->header);
      return;
    }
    auto IDX = [cols](int ix, int iy){ return iy * cols + ix; };
    auto in_bounds = [cols,rows](int x,int y){ return (x>=0 && x<cols && y>=0 && y<rows); };

    // 3) Hitung counts_raw per cell di seluruh bidang (TANPA smoothing)
    std::vector<int> counts_raw(rows*cols, 0);
    for (const auto& q : uv) {
      int ix = (int)std::floor((q.first  - umin)/cell);
      int iy = (int)std::floor((q.second - vmin)/cell);
      if (in_bounds(ix,iy)) counts_raw[IDX(ix,iy)]++;
    }

    // 4) Salinan untuk evaluasi fill-ratio (boleh dihaluskan)
    std::vector<int> counts = counts_raw;
    if (close_gaps_) {
      std::vector<int> counts2 = counts;
      for (int y=1;y<rows-1;++y){
        for (int x=1;x<cols-1;++x){
          int sum=0;
          for(int dy=-1;dy<=1;++dy)
            for(int dx=-1;dx<=1;++dx)
              sum += (counts[IDX(x+dx,y+dy)]>0);
          if (sum>=5 && counts[IDX(x,y)]==0) counts2[IDX(x,y)]=1; // isi celah kecil
        }
      }
      counts.swap(counts2);
    }

    // 5) Offsets disk aman (satuan sel)
    const int R_safe_cells = std::max(1, (int)std::round(r_safe / cell));
    std::vector<std::pair<int,int>> disk_offsets_safe;
    disk_offsets_safe.reserve((2*R_safe_cells+1)*(2*R_safe_cells+1));
    for (int dy=-R_safe_cells; dy<=R_safe_cells; ++dy)
      for (int dx=-R_safe_cells; dx<=R_safe_cells; ++dx)
        if (dx*dx + dy*dy <= R_safe_cells*R_safe_cells)
          disk_offsets_safe.emplace_back(dx,dy);
    const int disk_area_safe = (int)disk_offsets_safe.size();

    // 6) Kandidat pusat: cakupan (fill ratio) terpenuhi & tidak mepet tepi
    std::vector<uint8_t> safe_center(rows*cols, 0);
    for (int iy=0; iy<rows; ++iy) {
      for (int ix=0; ix<cols; ++ix) {
        if (ix < R_safe_cells || ix >= cols - R_safe_cells ||
            iy < R_safe_cells || iy >= rows - R_safe_cells) continue;

        int filled_cells=0;
        for (const auto& o : disk_offsets_safe) {
          const int jx = ix + o.first, jy = iy + o.second;
          if (!in_bounds(jx,jy)) continue;
          if (counts[IDX(jx,jy)] >= min_pts_cell_) filled_cells++;
        }
        const float ratio = (float)filled_cells / (float)disk_area_safe;
        if (ratio >= (float)fill_ratio_req_) safe_center[IDX(ix,iy)] = 1;
      }
    }

    // 7) Peta obstacle dari counts_raw (TANPA smoothing) + inflate (dilasi) 3x3
    std::vector<uint8_t> obstacle(rows*cols, 0);
    for (int iy=0; iy<rows; ++iy)
      for (int ix=0; ix<cols; ++ix)
        obstacle[IDX(ix,iy)] = (counts_raw[IDX(ix,iy)] < min_pts_cell_) ? 1 : 0;

    auto dilate_once = [&](std::vector<uint8_t>& src){
      std::vector<uint8_t> dst = src;
      for (int y=0; y<rows; ++y) {
        for (int x=0; x<cols; ++x) {
          if (src[IDX(x,y)]) continue;
          for (int dy=-1; dy<=1; ++dy) for (int dx=-1; dx<=1; ++dx) {
            int nx = x+dx, ny = y+dy;
            if (!in_bounds(nx,ny)) continue;
            if (src[IDX(nx,ny)]) { dst[IDX(x,y)] = 1; goto next_pix; }
          }
          next_pix: ;
        }
      }
      src.swap(dst);
    };
    for (int it=0; it<inflate_cells_; ++it) dilate_once(obstacle);

    // 8) Distance transform (BFS 8-neigh) dari obstacle
    const int INF = 1e9;
    std::vector<int> dist(rows*cols, INF);
    std::deque<std::pair<int,int>> q;
    for (int iy=0; iy<rows; ++iy)
      for (int ix=0; ix<cols; ++ix)
        if (obstacle[IDX(ix,iy)]) { dist[IDX(ix,iy)] = 0; q.emplace_back(ix,iy); }

    if (q.empty()) std::fill(dist.begin(), dist.end(), std::max(rows,cols)*2);

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

    // 9) Pilih pusat terbaik (clearance terbesar; tie-break paling tengah bbox)
    //    + VETO: tidak boleh ada obstacle di dalam disk aman
    const float cx_cell = (float)cols * 0.5f, cy_cell = (float)rows * 0.5f;
    int best_ix=-1, best_iy=-1;
    float best_clearance=-1e9f, best_center_d2=1e30f;

    for (int iy=0; iy<rows; ++iy) {
      for (int ix=0; ix<cols; ++ix) {
        if (!safe_center[IDX(ix,iy)]) continue;

        // VETO: tolak kandidat jika ada satu sel obstacle dalam disk
        bool obstacle_free = true;
        for (const auto& o : disk_offsets_safe) {
          int jx = ix + o.first, jy = iy + o.second;
          if (!in_bounds(jx,jy) || obstacle[IDX(jx,jy)]) { obstacle_free = false; break; }
        }
        if (!obstacle_free) continue;

        float clearance_m = dist[IDX(ix,iy)] * cell - r_safe;
        float dcx = ((float)ix + 0.5f - cx_cell) * cell;
        float dcy = ((float)iy + 0.5f - cy_cell) * cell;
        float center_d2 = dcx*dcx + dcy*dcy;

        if (clearance_m > best_clearance + 1e-6f ||
           (std::abs(clearance_m - best_clearance) <= 1e-6f && center_d2 < best_center_d2)) {
          best_clearance = clearance_m;
          best_center_d2 = center_d2;
          best_ix=ix; best_iy=iy;
        }
      }
    }

    if (best_ix < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No safe center found.");
      publishOnlySafe(*cloud, uv, false, 0.f, 0.f, 0.f, msg->header);
      return;
    }

    // 10) Pusat terpilih → koordinat (u,v)
    const float cu = umin + ((float)best_ix + 0.5f) * cell;
    const float cv = vmin + ((float)best_iy + 0.5f) * cell;

    // 11) Publish HANYA titik yang berada di disk aman (u-c <= r_safe)
    publishOnlySafe(*cloud, uv, true, cu, cv, r_safe, msg->header);
  }

  void publishOnlySafe(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                       const std::vector<std::pair<float,float>>& uv,
                       bool have_center, float cu, float cv, float r_safe,
                       const std_msgs::msg::Header& hdr)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr safe_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    safe_rgb->reserve(cloud.size());

    size_t kept = 0;
    if (have_center) {
      const float r2 = r_safe * r_safe;
      for (size_t i=0;i<cloud.size();++i) {
        float du = uv[i].first  - cu;
        float dv = uv[i].second - cv;
        if (du*du + dv*dv <= r2) {
          pcl::PointXYZRGB q;
          q.x = cloud[i].x; q.y = cloud[i].y; q.z = cloud[i].z;
          q.r = 30; q.g = 255; q.b = 30; // hijau
          safe_rgb->push_back(q);
          ++kept;
        }
      }
    }

    // Set metadata agar RViz happy
    safe_rgb->width  = (uint32_t)safe_rgb->size();
    safe_rgb->height = 1;
    safe_rgb->is_dense = false;

    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*safe_rgb, out);  // field 'rgb' ikut
    out.header = hdr;
    pub_->publish(out);

    RCLCPP_INFO(get_logger(), "Published safe points: %zu", kept);
  }

  // params & IO
  std::string input_topic_, output_topic_, plane_axes_;
  double safe_diameter_, grid_cell_, fill_ratio_req_;
  int    min_pts_cell_, inflate_cells_;
  bool   close_gaps_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafeLandingColor>());
  rclcpp::shutdown();
  return 0;
}
