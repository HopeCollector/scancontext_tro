#include <fmt/format.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <CLI/CLI.hpp>
#include <deque>
#include <execution>
#include <fkYAML/node.hpp>

#include "Scancontext.h"
#include "expr-utils/data_loader.hh"

namespace {
using PointT = SCPointType;
using CloudT = pcl::PointCloud<PointT>;

std::string config_path{""};

struct Config {
  std::string dataset_type{""};
  std::string lidar_path{""};
  std::string pose_path{""};
  std::string calib_path{""};
  std::string output_path{""};
  size_t cache_num{0};
};

int parse_param(int argc, char** argv) {
  CLI::App app{"STDesc expriement"};
  app.add_option("-c,--config", config_path, "Path to the config file")
      ->required();
  CLI11_PARSE(app, argc, argv);
  return 0;
}

Config load_config(const std::string& config_path) {
  Config ret;
  fkyaml::node node;
  {
    std::ifstream file(config_path);
    node = fkyaml::node::deserialize(file);
  }

  // Parameters of dataset
  ret.dataset_type = node["dataset"].get_value<std::string>();
  ret.lidar_path = node["lidar"].get_value<std::string>();
  ret.pose_path = node["pose"].get_value<std::string>();
  ret.calib_path = node["calib"].get_value<std::string>();
  ret.output_path = node["output"].get_value<std::string>();
  ret.cache_num = node["cache_num"].get_value<size_t>();
  return ret;
}

struct Result {
  size_t key_frame_id;
  size_t loop_frame_id;
  double score;
  double iou;
  Eigen::Vector3f center;

  Result(size_t id1, size_t id2, double score)
      : key_frame_id(id1),
        loop_frame_id(id2),
        score(score),
        iou(-1.0),
        center() {}

  friend std::ostream& operator<<(std::ostream& os, const Result& res) {
    os << res.key_frame_id << "," << res.loop_frame_id << "," << res.score
       << "," << res.iou << "," << res.center.x() << "," << res.center.y()
       << "," << res.center.z();
    return os;
  }
};

double iou(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cld1,
           pcl::PointCloud<pcl::PointXYZI>::ConstPtr cld2) {
  pcl::KdTreeFLANN<pcl::PointXYZI> tree;
  tree.setInputCloud(cld1);
  std::vector<bool> marks1(cld1->size(), false);
  std::vector<bool> marks2(cld2->size(), false);

  // search the near points in 0.5m
  for (size_t i = 0; i < cld2->size(); i++) {
    const auto& p = cld2->at(i);
    std::vector<int> indices;
    std::vector<float> distances;
    tree.radiusSearch(p, 0.5, indices, distances);
    if (indices.empty()) {
      continue;
    }
    // mark the indices
    for (const auto& idx : indices) {
      marks1[idx] = true;
    }
    marks2[i] = true;
  }

  // calculate the iou, no need to minus the intersection, because this is point
  // num, not volume
  size_t inter = std::count(marks1.begin(), marks1.end(), true) +
                 std::count(marks2.begin(), marks2.end(), true);
  return double(inter) / double(cld1->size() + cld2->size());
}

pcl::PointCloud<pcl::PointXYZI>::Ptr cache_cloud(
    utils::data_loader::Loader::Ptr loader, size_t start, size_t len) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr ret(new pcl::PointCloud<pcl::PointXYZI>);
  for (int i = start; i < start + len; i++) {
    auto tmp_cld = loader->seq(i, true);
    if (!tmp_cld) break;
    *ret += *tmp_cld;
  }
  return ret;
}

Eigen::Affine3f vec2transform(const Eigen::Vector4f& vec) {
  Eigen::Affine3f ret = Eigen::Affine3f::Identity();
  ret.translation() << vec[0], vec[1], vec[2];
  return ret;
}
}  // namespace

int main(int argc, char** argv) {
  // parse args
  parse_param(argc, argv);
  auto cfg = load_config(config_path);

  // data loader
  auto loader = utils::create_loader(cfg.dataset_type, cfg.lidar_path,
                                     cfg.pose_path, cfg.calib_path);

  // Scan Context Manager
  SCManager sc_manager;

  // running
  size_t results_sz{0};
  std::vector<Result> results;
  results.reserve(1e5);
  std::vector<std::pair<Eigen::Vector4f, CloudT::Ptr>> clouds;
  for (size_t i = 0; i < loader->size(); i += cfg.cache_num) {
    // load point cloud in local frame
    auto cur_cld = cache_cloud(loader, i, cfg.cache_num);
    Eigen::Vector4f center;
    pcl::compute3DCentroid(*cur_cld, center);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << -center[0], -center[1], -center[2];
    pcl::transformPointCloud(*cur_cld, *cur_cld, transform);

    // downsample
    CloudT::Ptr cur_ds_cld(new CloudT);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cur_cld);
    vg.setLeafSize(0.1, 0.1, 0.1);
    vg.filter(*cur_ds_cld);
    clouds.emplace_back(center, cur_ds_cld);

    // do the Scan Context things
    sc_manager.makeAndSaveScancontextAndKeys(*cur_cld);
    sc_manager.detectLoopClosureID();
    for (const auto& scres : sc_manager.results) {
      size_t id = i / 10;
      if (!results.empty()) {
        const auto& last = results.back();
        if (scres.loop_frame_id == last.loop_frame_id &&
            id == last.key_frame_id)
          continue;
      }
      results.emplace_back(id, size_t(scres.loop_frame_id),
                           1.0 / (1.0 + scres.dist));
    }
  }

  // transform the results
  std::for_each(std::execution::par_unseq, results.begin(), results.end(),
                [&clouds](Result& res) {
                  // transform the point cloud 1 to global frame
                  auto trans1 = vec2transform(clouds[res.key_frame_id].first);
                  CloudT::Ptr cld1{new CloudT};
                  pcl::transformPointCloud(*clouds[res.key_frame_id].second,
                                           *cld1, trans1);

                  // transform the point cloud 2 to global frame
                  auto trans2 = vec2transform(clouds[res.loop_frame_id].first);
                  CloudT::Ptr cld2{new CloudT};
                  pcl::transformPointCloud(*clouds[res.loop_frame_id].second,
                                           *cld2, trans2);

                  // iou
                  res.iou = iou(cld1, cld2);

                  // center
                  *cld1 += *cld2;
                  Eigen::Vector4f ctr;
                  pcl::compute3DCentroid(*cld1, ctr);
                  res.center = ctr.head<3>();
                });

  // save results
  std::ofstream file(cfg.output_path);
  if (!file.is_open()) {
    fmt::print("Cannot open file: {}\n", cfg.output_path);
    return -1;
  }
  for (const auto& res : results) {
    file << res << std::endl;
  }
  return 0;
}