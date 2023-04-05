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
#include <open3d/Open3D.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>

#include "ERASOR.h"
#include "utils.h"
#include "timing.hpp"

namespace erasor {

template <typename PointT>
class MapUpdater {
 public:
  MapUpdater(const std::string config_file_path);
  virtual ~MapUpdater() = default;

  YAML::Node yconfig;
  void setConfig();
  void saveMap(std::string const& file_path);

  void setRawMap(typename pcl::PointCloud<PointT>::Ptr const& raw_map);
  void run(typename pcl::PointCloud<PointT>::Ptr const& single_pc);
  
  benchmark::ERASOR erasor;
  ufo::Timing timing;
  
 private:
  common::Config cfg_;
  typename pcl::PointCloud<PointT>::Ptr query_voi_;
  typename pcl::PointCloud<PointT>::Ptr map_voi_;
  typename pcl::PointCloud<PointT>::Ptr map_outskirts_;
  typename pcl::PointCloud<PointT>::Ptr map_arranged_;

  /*** Outputs of ERASOR
   * map_filtered_ = map_static_estimate + map_egocentric_complement
   */
  typename pcl::PointCloud<PointT>::Ptr map_static_estimate_;
  typename pcl::PointCloud<PointT>::Ptr map_staticAdynamic;
  typename pcl::PointCloud<PointT>::Ptr map_filtered_;
  typename pcl::PointCloud<PointT>::Ptr map_egocentric_complement_;

  void fetch_VoI(double x_criterion, double y_criterion,
                 pcl::PointCloud<PointT>& query_pcd);
                 
};

}  // namespace erasor
