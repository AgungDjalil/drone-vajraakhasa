#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/extract_indices.h>

#include <Eigen/Dense>
#include <random>
#include <unordered_map>
#include <limits>

// -------------------------------
// Minimal Growing Neural Gas (3D)
// -------------------------------
struct GNGNode {
  Eigen::Vector3f w;     // bobot (posisi di ruang fitur)
  float error = 0.f;     // error akumulatif
  std::unordered_map<int,int> neigh_age; // tetangga -> usia edge
};

struct GNGParams {
  int   max_nodes      = 40;
  int   max_age        = 50;
  int   lambda_insert  = 100;   // tiap N langkah sisip simpul
  float eps_b          = 0.05f; // learning winner
  float eps_n          = 0.005f;// learning neighbor
  float alpha          = 0.5f;  // pengurangan error saat sisip
  float d              = 0.995f;// decay error per langkah
  int   train_steps    = 5000;  // jumlah langkah pelatihan
};

class GrowingNeuralGas3D {
public:
  explicit GrowingNeuralGas3D(const GNGParams& p) : P(p) {}

  void train(const std::vector<Eigen::Vector3f>& X, unsigned seed=1) {
    if (X.size() < 2) return;
    std::mt19937 rng(seed);
    std::uniform_int_distribution<size_t> pick(0, X.size()-1);

    nodes_.clear();
    // inisialisasi dua node acak
    GNGNode a, b;
    a.w = X[pick(rng)];
    b.w = X[pick(rng)];
    nodes_.push_back(a);
    nodes_.push_back(b);

    for (int t=1; t<=P.train_steps; ++t) {
      const Eigen::Vector3f& x = X[pick(rng)];

      // cari dua node terdekat (s1 = winner, s2 = runner up)
      int s1=-1, s2=-1; float d1=1e30f, d2=1e30f;
      for (int i=0;i<(int)nodes_.size();++i) {
        float d = (x - nodes_[i].w).squaredNorm();
        if (d < d1) { d2=d1; s2=s1; d1=d; s1=i; }
        else if (d < d2) { d2=d; s2=i; }
      }

      // naikkan usia edge dari s1
      for (auto &kv : nodes_[s1].neigh_age) kv.second++;

      // update error winner + geser winner & neighbors
      nodes_[s1].error += d1;
      nodes_[s1].w     += P.eps_b * (x - nodes_[s1].w);
      for (auto &kv : nodes_[s1].neigh_age) {
        int j = kv.first;
        nodes_[j].w += P.eps_n * (x - nodes_[j].w);
      }

      // hubungkan s1 <-> s2 (reset usia)
      connect_(s1, s2);

      // buang edge terlalu tua
      prune_old_edges_();

      // sisip node tiap lambda_insert langkah
      if ( (t % P.lambda_insert)==0 && (int)nodes_.size()<P.max_nodes ) {
        // q = node dgn error max
        int q=0; for (int i=1;i<(int)nodes_.size();++i)
          if (nodes_[i].error > nodes_[q].error) q=i;

        // f = tetangga q dengan error max
        int f = -1; float maxerr = -1.f;
        for (auto &kv : nodes_[q].neigh_age) {
          int j = kv.first;
          if (nodes_[j].error > maxerr) { maxerr = nodes_[j].error; f=j; }
        }
        if (f<0) continue;

        // r = node baru di tengah q-f
        GNGNode r;
        r.w = 0.5f*(nodes_[q].w + nodes_[f].w);
        r.error = (nodes_[q].error + nodes_[f].error)*0.5f;
        nodes_.push_back(r);
        int r_idx = (int)nodes_.size()-1;

        // hapus edge q-f, sambungkan q-r dan r-f
        disconnect_(q,f);
        connect_(q,r_idx);
        connect_(r_idx,f);

        // kurangi error q dan f
        nodes_[q].error *= P.alpha;
        nodes_[f].error *= P.alpha;
        nodes_[r_idx].error = nodes_[q].error; // atau max/avg; bebas
      }

      // decay error
      for (auto &nd : nodes_) nd.error *= P.d;
    }
  }

  // indeks node terdekat (untuk assign klaster)
  int nearest(const Eigen::Vector3f& x) const {
    int s=-1; float dmin=1e30f;
    for (int i=0;i<(int)nodes_.size();++i) {
      float d = (x - nodes_[i].w).squaredNorm();
      if (d < dmin) { dmin=d; s=i; }
    }
    return s;
  }

  const std::vector<GNGNode>& nodes() const { return nodes_; }

private:
  GNGParams P;
  std::vector<GNGNode> nodes_;

  void connect_(int i,int j){
    nodes_[i].neigh_age[j]=0;
    nodes_[j].neigh_age[i]=0;
  }
  void disconnect_(int i,int j){
    nodes_[i].neigh_age.erase(j);
    nodes_[j].neigh_age.erase(i);
  }
  void prune_old_edges_(){
    for (int i=0;i<(int)nodes_.size();++i) {
      auto it = nodes_[i].neigh_age.begin();
      while (it!=nodes_[i].neigh_age.end()){
        int j = it->first; int age = it->second;
        if (age > P.max_age) { // hapus edge tua
          nodes_[j].neigh_age.erase(i);
          it = nodes_[i].neigh_age.erase(it);
        } else ++it;
      }
    }
  }
};

// ----------------------------------------------------------
// ROS2 Node: GNG-based plane segmentation using surface normal
// ----------------------------------------------------------
class PlaneSegmentationGNG : public rclcpp::Node {
public:
  PlaneSegmentationGNG() : Node("plane_segmentation_gng") {
    // I/O
    declare_parameter<std::string>("input_topic",  "/circle_cloud");
    declare_parameter<std::string>("output_topic", "/segmented");
    // Preprocess
    declare_parameter<double>("z_min", -5.0);
    declare_parameter<double>("z_max",  5.0);
    declare_parameter<double>("leaf_size", 0.05);
    declare_parameter<int>("k_normals", 30);
    // GNG
    declare_parameter<int>("gng_max_nodes", 40);
    declare_parameter<int>("gng_train_steps", 5000);
    declare_parameter<int>("gng_max_age", 50);
    declare_parameter<int>("gng_lambda", 100);
    declare_parameter<double>("gng_eps_b", 0.05);
    declare_parameter<double>("gng_eps_n", 0.005);
    declare_parameter<double>("gng_alpha", 0.5);
    declare_parameter<double>("gng_d", 0.995);
    // Plane validation
    declare_parameter<double>("plane_dist_thresh", 0.09);   // meter (ketebalan)
    declare_parameter<double>("planarity_ratio",  0.03);   // lambda_min / trace
    declare_parameter<double>("min_inlier_ratio", 0.60);    // % inlier utk menerima klaster

    const auto in  = get_parameter("input_topic").as_string();
    const auto out = get_parameter("output_topic").as_string();

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      in, rclcpp::SensorDataQoS(),
      std::bind(&PlaneSegmentationGNG::cb, this, std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(out, 10);
    pub_debug_ = create_publisher<sensor_msgs::msg::PointCloud2>(out + "_debug", 10);

    RCLCPP_INFO(get_logger(), "GNG plane seg: %s -> %s", in.c_str(), out.c_str());
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_debug_;

  void cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // --- input ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) return;

    const float zmin  = (float)get_parameter("z_min").as_double();
    const float zmax  = (float)get_parameter("z_max").as_double();
    const float leaf  = (float)get_parameter("leaf_size").as_double();
    const int   k     = get_parameter("k_normals").as_int();

    // --- ROI in z ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr roi(new pcl::PointCloud<pcl::PointXYZ>);
    {
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud(cloud);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(zmin,zmax);
      pass.filter(*roi);
    }
    if (roi->empty()) return;

    // --- Voxel downsample ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>);
    {
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud(roi);
      vg.setLeafSize(leaf,leaf,leaf);
      vg.filter(*down);
    }
    if (down->empty()) return;

    // --- Normal estimation (OMP) ---
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    {
      pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
      auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
      ne.setInputCloud(down);
      ne.setSearchMethod(tree);
      ne.setKSearch(std::max(6, k));
      ne.compute(*normals);
    }

    // --- Siapkan data fitur utk GNG: vektor normal unit (nx,ny,nz) ---
    std::vector<Eigen::Vector3f> feats;
    feats.reserve(normals->size());
    for (const auto& n : normals->points) {
      Eigen::Vector3f v(n.normal_x, n.normal_y, n.normal_z);
      float L = v.norm();
      if (std::isfinite(L) && L > 1e-6f) feats.emplace_back(v / L);
    }
    if (feats.size() < 10) {
      RCLCPP_WARN(get_logger(), "Too few normals for GNG (%zu).", feats.size());
      return;
    }

    // --- Train GNG ---
    GNGParams gp;
    gp.max_nodes     = get_parameter("gng_max_nodes").as_int();
    gp.train_steps   = get_parameter("gng_train_steps").as_int();
    gp.max_age       = get_parameter("gng_max_age").as_int();
    gp.lambda_insert = get_parameter("gng_lambda").as_int();
    gp.eps_b         = (float)get_parameter("gng_eps_b").as_double();
    gp.eps_n         = (float)get_parameter("gng_eps_n").as_double();
    gp.alpha         = (float)get_parameter("gng_alpha").as_double();
    gp.d             = (float)get_parameter("gng_d").as_double();

    GrowingNeuralGas3D gng(gp);
    gng.train(feats, /*seed*/1);
    const auto& nodes = gng.nodes();
    if (nodes.size()<2) {
      RCLCPP_WARN(get_logger(), "GNG produced too few nodes.");
      return;
    }

    // --- Assign tiap titik ke node terdekat (klaster by normal) ---
    std::vector<int> cluster_idx(down->size(), -1);
    for (size_t i=0;i<down->size();++i) {
      const auto& n = normals->points[i];
      Eigen::Vector3f v(n.normal_x, n.normal_y, n.normal_z);
      float L = v.norm(); if (L<=1e-6f || !std::isfinite(L)) continue;
      v/=L;
      cluster_idx[i] = gng.nearest(v);
    }

    // --- Kumpulkan titik per klaster & validasi planarity ---
    const float dist_thr = (float)get_parameter("plane_dist_thresh").as_double();
    const float planarity_ratio = (float)get_parameter("planarity_ratio").as_double();
    const float min_inlier_ratio = (float)get_parameter("min_inlier_ratio").as_double();

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters(nodes.size());
    for (auto& c : clusters) c.reset(new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i=0;i<down->size();++i) {
      int cid = cluster_idx[i];
      if (cid>=0) clusters[cid]->push_back(down->points[i]);
    }

    // Hasil akhir: gabungkan semua klaster yang lolos syarat plane
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_col(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_only(new pcl::PointCloud<pcl::PointXYZ>);

    auto color_for = [](int i)->Eigen::Vector3i {
      // palet sederhana
      static const int C[12][3] = {
        {230,25,75},{60,180,75},{255,225,25},{0,130,200},
        {245,130,48},{145,30,180},{70,240,240},{240,50,230},
        {210,245,60},{250,190,190},{0,128,128},{230,190,255}
      };
      auto c = C[i%12]; return Eigen::Vector3i(c[0],c[1],c[2]);
    };

    for (size_t cid=0; cid<clusters.size(); ++cid) {
      auto& pc = clusters[cid];
      if (pc->size() < 200) continue; // terlalu kecil

      // PCA plane fit
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*pc, centroid);
      Eigen::Matrix3f cov; pcl::computeCovarianceMatrixNormalized(*pc, centroid, cov);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
      Eigen::Vector3f eval = es.eigenvalues();
      Eigen::Matrix3f evec = es.eigenvectors();
      // eigenvalue terkecil → normal
      int idx_min = 0; if (eval[1]<eval[idx_min]) idx_min=1; if (eval[2]<eval[idx_min]) idx_min=2;
      Eigen::Vector3f n = evec.col(idx_min).normalized();
      float curvature = eval[idx_min] / eval.sum();

      if (curvature > planarity_ratio) continue; // tidak planar

      // Inlier check berdasar jarak ke plane
      const Eigen::Vector3f c = centroid.head<3>();
      int inliers=0;
      for (const auto& p : pc->points) {
        Eigen::Vector3f v(p.x,p.y,p.z);
        float dist = std::abs( (v-c).dot(n) );
        if (dist <= dist_thr) inliers++;
      }
      float ratio = (float)inliers / (float)pc->size();
      if (ratio < min_inlier_ratio) continue;

      // tambahkan ke hasil plane_only dan debug coloring
      const auto col = color_for((int)cid);
      for (const auto& p : pc->points) {
        // debug warna
        pcl::PointXYZRGB q; q.x=p.x; q.y=p.y; q.z=p.z;
        q.r=col[0]; q.g=col[1]; q.b=col[2];
        debug_col->push_back(q);
      }
      // plane only = hanya inlier (opsional: masukkan semua)
      for (const auto& p : pc->points) {
        Eigen::Vector3f v(p.x,p.y,p.z);
        float dist = std::abs( (v-c).dot(n) );
        if (dist <= dist_thr) plane_only->push_back(p);
      }
    }

    // publish
    sensor_msgs::msg::PointCloud2 msg_plane, msg_dbg;
    pcl::toROSMsg(*plane_only, msg_plane);
    msg_plane.header = msg->header;
    pub_->publish(msg_plane);

    if (pub_debug_->get_subscription_count()>0) {
      pcl::toROSMsg(*debug_col, msg_dbg);
      msg_dbg.header = msg->header;
      pub_debug_->publish(msg_dbg);
    }
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlaneSegmentationGNG>());
  rclcpp::shutdown();
  return 0;
}