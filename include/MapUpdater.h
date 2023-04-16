/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * Only this file in under MIT License
 * Author: Kin ZHANG (https://kin-zhang.github.io/)
 * Date: 2023-04-04 23:19
 * Description: No ROS version, speed up the process
 *
 * Please reference official ERASOR paper for more details
 * This modified version is for the purpose of benchmarking no ROS! speed up! by
 * Kin, Reference the LICENSE file in origin repo.
 */

#pragma once

// function lib
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>

#include "ERASOR.h"
#include "timing.hpp"
#include "utils.h"

namespace erasor {

class MapUpdater {
 public:
  MapUpdater(const std::string config_file_path);
  virtual ~MapUpdater() = default;

  YAML::Node yconfig;
  void setConfig();
  void saveMap(std::string const& file_path);

  void setRawMap(pcl::PointCloud<PointT>::Ptr const& raw_map);
  void run(pcl::PointCloud<PointT>::Ptr const& single_pc);
  const common::Config getCfg() { return cfg_; }
  benchmark::ERASOR erasor;
  ufo::Timing timing;

 private:
  common::Config cfg_;
  pcl::PointCloud<PointT>::Ptr query_voi_;
  pcl::PointCloud<PointT>::Ptr map_voi_;
  pcl::PointCloud<PointT>::Ptr map_outskirts_;
  pcl::PointCloud<PointT>::Ptr map_arranged_;
  pcl::PointCloud<PointT>::Ptr map_arranged_global_, map_arranged_complement_;

  /*** Outputs of ERASOR
   * map_filtered_ = map_static_estimate + map_egocentric_complement
   */
  pcl::PointCloud<PointT>::Ptr map_static_estimate_;
  pcl::PointCloud<PointT>::Ptr map_staticAdynamic;
  pcl::PointCloud<PointT>::Ptr map_filtered_;
  pcl::PointCloud<PointT>::Ptr map_egocentric_complement_;

  void reassign_submap(double pose_x, double pose_y);
  void fetch_VoI(double x_criterion, double y_criterion,
                 pcl::PointCloud<PointT>& query_pcd);

  void set_submap(const pcl::PointCloud<pcl::PointXYZI>& map_global,
                  pcl::PointCloud<pcl::PointXYZI>& submap,
                  pcl::PointCloud<pcl::PointXYZI>& submap_complement, double x,
                  double y, double submap_size);

  double submap_size_ = 200.0;
  double submap_center_x_, submap_center_y_;
  int num_pcs_init_;
  bool is_submap_not_initialized_ = true;
};

}  // namespace erasor
