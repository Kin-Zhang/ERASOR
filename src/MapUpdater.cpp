/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * MIT License
 * Author: Kin ZHANG (https://kin-zhang.github.io/)
 * Date: 2023-04-04 23:09
 * Description: No ROS version, speed up the process
 *
 * Please reference official ERASOR paper for more details
 * This modified version is for the purpose of benchmarking no ROS! speed up! by Kin
 */
#include <glog/logging.h>

#include "MapUpdater.h"

namespace erasor {
MapUpdater::MapUpdater(const std::string config_file_path) {

	yconfig = YAML::LoadFile(config_file_path);
	MapUpdater::setConfig();

}

void MapUpdater::setConfig(){
    cfg_.query_voxel_size_ = yconfig["MapUpdater"]["query_voxel_size"].as<double>();
    cfg_.map_voxel_size_ = yconfig["MapUpdater"]["map_voxel_size"].as<double>();
    cfg_.removal_interval_ = yconfig["MapUpdater"]["removal_interval"].as<int>();
    cfg_.global_voxelization_period_ = yconfig["MapUpdater"]["voxelization_interval"].as<int>();

    cfg_.max_range_ = yconfig["erasor"]["max_range"].as<double>();
    cfg_.min_h_ = yconfig["erasor"]["min_h"].as<double>();
    cfg_.max_h_ = yconfig["erasor"]["max_h"].as<double>();
    cfg_.num_rings_ = yconfig["erasor"]["num_rings"].as<int>();
    cfg_.num_sectors_ = yconfig["erasor"]["num_sectors"].as<int>();
    cfg_.th_bin_max_h = yconfig["erasor"]["th_bin_max_h"].as<double>();
    cfg_.scan_ratio_threshold = yconfig["erasor"]["scan_ratio_threshold"].as<double>();


}

void MapUpdater::setRawMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& raw_map) {
    // TODO
}

void MapUpdater::run(const pcl::PointCloud<pcl::PointXYZI>::Ptr& single_pc) {
    // TODO
}

void MapUpdater::saveMap(const std::string& file_path) {
    // TODO
}

}  // namespace erasor