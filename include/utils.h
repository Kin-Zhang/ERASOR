
/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * Only this file in under MIT License
 * Author: Kin ZHANG (https://kin-zhang.github.io/)
 * Date: 2023-04-04 23:19
 * Description: Config header
 */


#pragma once
#include <iostream>
namespace common {
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

  double th_seeds_heights_ = 0.5;
  double th_dist_ = 0.05;
  int num_lprs_ = 10;
  int minimum_num_pts = 6;
  int iter_groundfilter_ = 3;
  int num_lowest_pts = 5;
  bool verbose_ = true;  // print out logs

  std::string mode = "naive";
};

}  // namespace common