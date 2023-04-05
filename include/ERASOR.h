
/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * Only this file in under MIT License
 * Author: Kin ZHANG (https://kin-zhang.github.io/)
 * Date: 2023-04-05 11:19
 * Description: No ROS version, speed up the process
 *
 * Please reference official ERASOR paper for more details
 * This modified version is for the purpose of benchmarking no ROS! speed up! by
 * Kin, Reference the LICENSE file in origin repo.
 */

#pragma once
#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "utils.h"

#define INF 10000000000000.0
#define PI 3.1415926535
#define ENOUGH_NUM 8000

#define EMPTY 0
#define MAP 1
#define PC_CURR 2
// COLORS:
// 0 -> BLUE
#define MAP_IS_HIGHER 0.5
#define CURR_IS_HIGHER 1.0
#define LITTLE_NUM 0.0  // For viz: blue - not activated
#define BLOCKED 0.8     // For viz

#define MERGE_BINS 0.25
#define NOT_ASSIGNED 0.0
// ground params

using namespace std;

struct Bin {
  double max_h;
  double min_h;
  double x;
  double y;
  double status;
  bool is_occupied;

  pcl::PointCloud<pcl::PointXYZI> points;
};

struct DynamicBinIdx {
  int r;
  int theta;
};

typedef vector<vector<Bin> > R_POD;
typedef vector<Bin> Ring;
namespace benchmark {
class ERASOR {
 public:
  ERASOR();
  virtual ~ERASOR() = default;

  void setConfig(common::Config& cfg);

  void set_inputs(const pcl::PointCloud<pcl::PointXYZI>& map_voi,
                  const pcl::PointCloud<pcl::PointXYZI>& query_voi,
                  const double x_cur, const double y_cur);
  void compare_vois_and_revert_ground_w_block();
  void get_static_estimate(pcl::PointCloud<pcl::PointXYZI>& arranged,
                           pcl::PointCloud<pcl::PointXYZI>& complement);
                           
 private:
  pcl::PointCloud<pcl::PointXYZI> piecewise_ground_, non_ground_;
  pcl::PointCloud<pcl::PointXYZI> ground_pc_, non_ground_pc_;
  void init(R_POD& r_pod);
  void extract_ground(pcl::PointCloud<pcl::PointXYZI>& src,
                      pcl::PointCloud<pcl::PointXYZI>& dst,
                      pcl::PointCloud<pcl::PointXYZI>& outliers);
  bool is_dynamic_obj_close(R_POD& r_pod_selected, int r_target, int theta_target);
  void extract_initial_seeds_(const pcl::PointCloud<pcl::PointXYZI>& p_sorted,
                              pcl::PointCloud<pcl::PointXYZI>& init_seeds);
  void estimate_plane_(const pcl::PointCloud<pcl::PointXYZI>& ground);

  void clear_bin(Bin& bin);
  void clear(pcl::PointCloud<pcl::PointXYZI>& pt_cloud);

  void pt2r_pod(const pcl::PointXYZI& pt, Bin& bin);

  void voi2r_pod(const pcl::PointCloud<pcl::PointXYZI>& src, R_POD& r_pod);

  void voi2r_pod(const pcl::PointCloud<pcl::PointXYZI>& src, R_POD& r_pod,
                 pcl::PointCloud<pcl::PointXYZI>& complement);

  double xy2theta(const double& x, const double& y);

  double xy2radius(const double& x, const double& y);
  void r_pod2pc(const R_POD& sc, pcl::PointCloud<pcl::PointXYZI>& pc);

  common::Config cfg_;
  R_POD r_pod_map;       // R_POD of Map
  R_POD r_pod_curr;      // R_POD of current pointcloud
  R_POD r_pod_selected;  // R_POD of current pointcloud
  Eigen::MatrixXf normal_;
  double th_dist_d_, d_;
  double ring_size;
  double sector_size;
  double x_cur_, y_cur_;
  pcl::PointCloud<pcl::PointXYZI> map_complement;
  pcl::PointCloud<pcl::PointXYZI> ground_viz;  // Visualized in pcs_v2!
};
}  // namespace benchmark
