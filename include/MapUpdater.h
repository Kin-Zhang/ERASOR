/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * MIT License
 * Author: Kin ZHANG (https://kin-zhang.github.io/)
 * Date: 2023-04-04 23:19
 * Description: No ROS version, speed up the process
 *
 * Please reference official ERASOR paper for more details
 * This modified version is for the purpose of benchmarking no ROS! speed up! by
 * Kin
 */

#pragma once

// function lib
#include <glog/logging.h>
#include <open3d/Open3D.h>
#include <yaml-cpp/yaml.h>

namespace erasor {

class MapUpdater {
 public:
  MapUpdater(const std::string config_file_path);
  virtual ~MapUpdater() = default;
  struct Config {
    /**< Parameters of MapUpdater*/
    double query_voxel_size_;
    double map_voxel_size_;
    int removal_interval_;
    int global_voxelization_period_;

    /**< Params. of Volume of Interest (VoI) */
    double max_range_;
    int num_rings_, num_sectors_;
    double min_h_, max_h_;
    double th_bin_max_h, scan_ratio_threshold;

    double submap_size_;
    double submap_center_x_;
    double submap_center_y_;
  };

  YAML::Node yconfig;
  void setConfig();

 private:
  Config cfg_;
};

}  // namespace erasor
