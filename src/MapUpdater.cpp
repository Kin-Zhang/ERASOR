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
#include "timer.h"

namespace erasor {
template <typename PointT>
MapUpdater<PointT>::MapUpdater(const std::string config_file_path) {

	yconfig = YAML::LoadFile(config_file_path);
	MapUpdater::setConfig();

    erasor.setConfig(cfg_);

    // reset
    map_static_estimate_.reset(new pcl::PointCloud<PointT>());
    map_egocentric_complement_.reset(new pcl::PointCloud<PointT>());
    map_filtered_.reset(new pcl::PointCloud<PointT>());
}
template <typename PointT>
void MapUpdater<PointT>::setConfig(){
    cfg_.query_voxel_size_ = yconfig["MapUpdater"]["query_voxel_size"].as<double>();
    cfg_.map_voxel_size_ = yconfig["MapUpdater"]["map_voxel_size"].as<double>();
    cfg_.removal_interval_ = yconfig["MapUpdater"]["removal_interval"].as<int>();
    cfg_.global_voxelization_period_ = yconfig["MapUpdater"]["voxelization_interval"].as<int>();

    cfg_.max_range_ = yconfig["erasor"]["max_range"].as<double>();
    cfg_.min_h_ = yconfig["erasor"]["min_h"].as<double>();
    cfg_.max_h_ = yconfig["erasor"]["max_h"].as<double>();
    cfg_.num_rings_ = yconfig["erasor"]["num_rings"].as<int>();
    cfg_.num_sectors_ = yconfig["erasor"]["num_sectors"].as<int>();
    LOG(INFO) << "number of rings: " << cfg_.num_rings_ << ", number of sectors: " << cfg_.num_sectors_;
    cfg_.th_bin_max_h = yconfig["erasor"]["th_bin_max_h"].as<double>();
    cfg_.scan_ratio_threshold = yconfig["erasor"]["scan_ratio_threshold"].as<double>();

    cfg_.minimum_num_pts = yconfig["erasor"]["minimum_num_pts"].as<int>();
    cfg_.iter_groundfilter_ = yconfig["erasor"]["gf_iter"].as<int>();

    cfg_.num_lprs_ = yconfig["erasor"]["gf_num_lpr"].as<int>();
    cfg_.th_seeds_heights_ = yconfig["erasor"]["gf_th_seeds_height"].as<double>();
    cfg_.th_dist_ = yconfig["erasor"]["gf_dist_thr"].as<double>();
    cfg_.verbose_ = yconfig["verbose"].as<bool>();
    // if yconfig have num_lowest_pts
    if (yconfig["erasor"]["num_lowest_pts"]) {
        cfg_.num_lowest_pts = yconfig["erasor"]["num_lowest_pts"].as<int>();
    }
}
template <typename PointT>
void MapUpdater<PointT>::setRawMap(typename pcl::PointCloud<PointT>::Ptr const& raw_map) {
    // copy raw map to map_arranged
    map_arranged_.reset(new pcl::PointCloud<PointT>());
    pcl::copyPointCloud(*raw_map, *map_arranged_);
}


template <typename PointT>
void MapUpdater<PointT>::run(typename pcl::PointCloud<PointT>::Ptr const& single_pc) {
    // read pose in VIEWPOINT Field in pcd
    float x_curr = single_pc->sensor_origin_[0];
    float y_curr = single_pc->sensor_origin_[1];
    LOG(INFO) << "x_curr: " << x_curr << ", y_curr: " << y_curr;
    TIC;
    fetch_VoI(x_curr, y_curr, *single_pc); // query_voi_ and map_voi_ are ready in the same world frame
    TOC("fetch_VoI", cfg_.verbose_);
    
    // TRE;
    LOG_IF(INFO, cfg_.verbose_) << "map voi size: " << map_voi_->size() << " query voi: " << query_voi_->size();
    erasor.set_inputs(*map_voi_, *query_voi_, x_curr, y_curr);
    erasor.compare_vois_and_revert_ground_w_block();
    erasor.get_static_estimate(*map_static_estimate_, *map_egocentric_complement_);
    LOG_IF(INFO, cfg_.verbose_) << "Static pts num: " << map_static_estimate_->size();

    *map_arranged_ = *map_static_estimate_ + *map_egocentric_complement_;
}
template <typename PointT>
void MapUpdater<PointT>::saveMap(std::string const& file_path) {
    // save map_static_estimate_
    if (map_arranged_->size() == 0) {
        LOG(WARNING) << "map_static_estimate_ is empty, no map is saved";
        return;
    }
    pcl::io::savePCDFileBinary(file_path, *map_arranged_);
}

template <typename PointT>
void MapUpdater<PointT>::fetch_VoI(
        double x_criterion, double y_criterion, pcl::PointCloud<PointT> &query_pcd) {

    query_voi_.reset(new pcl::PointCloud<PointT>());
    map_voi_.reset(new pcl::PointCloud<PointT>());
    map_outskirts_.reset(new pcl::PointCloud<PointT>());

    if (cfg_.mode == "naive") {
        double max_dist_square = pow(cfg_.max_range_, 2);
        // find query voi
        for (auto const &pt : query_pcd.points) {
            double dist_square = pow(pt.x - x_criterion, 2) + pow(pt.y - y_criterion, 2);
            if (dist_square < max_dist_square) {
                query_voi_->points.emplace_back(pt);
            }
        }

        // find map voi
        for (auto const &pt : map_arranged_->points) {
            double dist_square = pow(pt.x - x_criterion, 2) + pow(pt.y - y_criterion, 2);
            if (dist_square < max_dist_square) {
                map_voi_ -> points.emplace_back(pt);
            }
            else {
                map_outskirts_-> points.emplace_back(pt);
            }
        }

    }
    //  else if (mode == "kdtree") {
    //     PointT searchPoint;
    //     searchPoint.x = x_criterion;
    //     searchPoint.x = y_criterion;
    //     searchPoint.z = 0.5;
    //     std::cout << "\033[1;32mKDTREE mode " << (*map_arranged_).points.size() << "\033[0m" << std::endl;
    //     std::vector<int>                     pointIdxRadiusSearch;
    //     std::vector<float>                   pointRadiusSquaredDistance;
    //     pcl::KdTreeFLANN<PointT>     kdtree;
    //     pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    //     *cloud = *map_arranged_;
    //     kdtree.setInputCloud(cloud);

    //     if (kdtree.radiusSearch(searchPoint, max_range_ + 0.5, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
    //         // To get outlier
    //         std::vector<char> isTrue(map_arranged_->points.size(), false);
    //         std::cout << "what?? " << pointIdxRadiusSearch.size();
    //         std::cout << "    " << isTrue.size();
    //         for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
    //             auto pt = (*cloud)[pointIdxRadiusSearch[i]];
    //             map_voi_wrt_origin_->points.emplace_back(pt);
    //             isTrue[pointIdxRadiusSearch[i]] = true;
    //         }
    //         for (size_t      j = 0; j < map_arranged_->points.size(); ++j) {
    //             if (!isTrue[j]) {
    //                 outskirts.push_back(map_arranged_->points[j]);
    //             }
    //         }
    //     }
    // }
    LOG_IF(INFO, cfg_.verbose_) << map_arranged_->points.size() << " points in the map";
}

// template class MapUpdater<pcl::PointXYZ>;
template class MapUpdater<pcl::PointXYZI>;
// template class MapUpdater<pcl::PointXYZRGB>;

}  // namespace erasor