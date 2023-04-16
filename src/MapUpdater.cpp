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
#include <pcl/filters/voxel_grid.h>
#include "MapUpdater.h"


namespace erasor {

#define NUM_PTS_LARGE_ENOUGH 200000
#define NUM_PTS_LARGE_ENOUGH_FOR_MAP 20000000

MapUpdater::MapUpdater(const std::string config_file_path) {

	yconfig = YAML::LoadFile(config_file_path);
	MapUpdater::setConfig();

    erasor.setConfig(cfg_);

    // reset
    map_static_estimate_.reset(new pcl::PointCloud<PointT>());
    map_egocentric_complement_.reset(new pcl::PointCloud<PointT>());
    map_staticAdynamic.reset(new pcl::PointCloud<PointT>());
    map_filtered_.reset(new pcl::PointCloud<PointT>());
    map_arranged_global_.reset(new pcl::PointCloud<PointT>());
    map_arranged_complement_.reset(new pcl::PointCloud<PointT>());
}

void MapUpdater::setConfig(){
    cfg_.tf_z = yconfig["tf"]["lidar2body"][2].as<double>();

    cfg_.query_voxel_size_ = yconfig["MapUpdater"]["query_voxel_size"].as<double>();
    cfg_.map_voxel_size_ = yconfig["MapUpdater"]["map_voxel_size"].as<double>();
    cfg_.removal_interval_ = yconfig["MapUpdater"]["removal_interval"].as<int>();
    cfg_.global_voxelization_period_ = yconfig["MapUpdater"]["voxelization_interval"].as<int>();

    cfg_.max_range_ = yconfig["erasor"]["max_range"].as<double>();
    cfg_.min_h_ = yconfig["erasor"]["min_h"].as<double>();// - cfg_.tf_z; // since need to account for the sensor height
    cfg_.max_h_ = yconfig["erasor"]["max_h"].as<double>();// - cfg_.tf_z;
    cfg_.num_rings_ = yconfig["erasor"]["num_rings"].as<int>();
    cfg_.num_sectors_ = yconfig["erasor"]["num_sectors"].as<int>();
    
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

void VoxelPointCloud(const pcl::PointCloud<PointT>::Ptr& cloud, pcl::PointCloud<PointT>::Ptr& cloud_voxelized, const double voxel_size) {
    if(voxel_size <= 0.001) {
        *cloud_voxelized = *cloud;
        LOG(WARNING) << "Voxel size is too small, no need to voxel grid filter!";
        return;
    }
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel_grid.filter(*cloud_voxelized);
}

void MapUpdater::setRawMap(pcl::PointCloud<PointT>::Ptr const& raw_map) {
    // copy raw map to map_arranged
    timing.start("0. Read RawMap  ");
    map_arranged_.reset(new pcl::PointCloud<PointT>());
    VoxelPointCloud(raw_map, map_arranged_, cfg_.map_voxel_size_);
    num_pcs_init_ = map_arranged_->points.size();
    if(cfg_.is_large_scale_) {
        map_arranged_global_->reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);
        *map_arranged_global_ = *map_arranged_;
        LOG_IF(INFO, cfg_.verbose_) << "Large-scale mode is on!";
        LOG_IF(INFO, cfg_.verbose_) << "Submap size is: " << submap_size_;
    }
    timing.stop("0. Read RawMap  ");
}



void MapUpdater::run(pcl::PointCloud<PointT>::Ptr const& single_pc) {
    
    pcl::PointCloud<PointT>::Ptr filter_pc(new pcl::PointCloud<PointT>());
    VoxelPointCloud(single_pc, filter_pc, cfg_.query_voxel_size_);
    // read pose in VIEWPOINT Field in pcd
    float x_curr = single_pc->sensor_origin_[0];
    float y_curr = single_pc->sensor_origin_[1];
    float z_curr = single_pc->sensor_origin_[2];

    if (cfg_.is_large_scale_) {
        reassign_submap(x_curr, y_curr);
    }

    timing.start("1. Fetch VoI    ");
    LOG_IF(INFO, cfg_.verbose_) << "x_curr: " << x_curr << ", y_curr: " << y_curr;
    fetch_VoI(x_curr, y_curr, *filter_pc); // query_voi_ and map_voi_ are ready in the same world frame
    timing.stop("1. Fetch VoI    ");
    LOG_IF(INFO, cfg_.verbose_) << ANSI_CYAN <<  "Fetch VoI time: " << timing.lastSeconds("1. Fetch VoI    ") << ANSI_RESET;
    LOG_IF(INFO, cfg_.verbose_) << "map voi size: " << map_voi_->size() << " query voi: " << query_voi_->size();

    
    erasor.setCenter(x_curr, y_curr, z_curr);
    erasor.set_inputs(*map_voi_, *query_voi_);
    timing.start("2. Compare VoI  ");
    erasor.compare_vois_and_revert_ground_w_block();
    timing.stop("2. Compare VoI  ");
    LOG_IF(INFO, cfg_.verbose_) << ANSI_CYAN << "Compare VoI time: " << timing.lastSeconds("2. Compare VoI  ") << ANSI_RESET;
    timing.start("3. Get StaticPts");
    erasor.get_static_estimate(*map_static_estimate_, *map_staticAdynamic, *map_egocentric_complement_);
    timing.stop("3. Get StaticPts");

    LOG_IF(INFO, cfg_.verbose_) << ANSI_CYAN << "Get StaticPts time: " << timing.lastSeconds("3. Get StaticPts") << ANSI_RESET;
    LOG_IF(INFO, cfg_.verbose_) << "Static pts num: " << map_static_estimate_->size();

    *map_arranged_ = *map_static_estimate_ + *map_outskirts_ + *map_egocentric_complement_;
}

void MapUpdater::saveMap(std::string const& folder_path) {
    pcl::PointCloud<PointT>::Ptr ptr_src(new pcl::PointCloud<PointT>);
    ptr_src->reserve(num_pcs_init_);

    if (cfg_.is_large_scale_) {
        LOG(INFO) << "Merging submap and complements...";
        *ptr_src = *map_arranged_ + *map_arranged_complement_;
    } else {
        *ptr_src = *map_arranged_;
    }
    // save map_static_estimate_
    if (ptr_src->size() == 0) {
        LOG(WARNING) << "map_static_estimate_ is empty, no map is saved";
        return;
    }
    if (map_staticAdynamic->size() > 0 && cfg_.replace_intensity){
        pcl::io::savePCDFileBinary(folder_path + "/erasor_output_whole.pcd", *map_staticAdynamic+*ptr_src);
    }
    pcl::io::savePCDFileBinary(folder_path + "/erasor_output.pcd", *ptr_src);
}


void MapUpdater::fetch_VoI(
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
        for (auto &pt : map_arranged_->points) {
            double dist_square = pow(pt.x - x_criterion, 2) + pow(pt.y - y_criterion, 2);
            if (dist_square < max_dist_square) {
                map_voi_ -> points.emplace_back(pt);
            }
            else{
                if(cfg_.replace_intensity)
                    pt.intensity = 0;
                map_outskirts_-> points.emplace_back(pt);
            }
        }
    }
    LOG_IF(INFO, cfg_.verbose_) << map_arranged_->points.size() << " points in the map";
}

void MapUpdater::reassign_submap(double pose_x, double pose_y){
    if (is_submap_not_initialized_) {
        set_submap(*map_arranged_global_, *map_arranged_, *map_arranged_complement_, pose_x, pose_y, submap_size_);
        submap_center_x_ = pose_x;
        submap_center_y_ = pose_y;
        is_submap_not_initialized_ = false;

        LOG_IF(INFO, cfg_.verbose_) << "\033[1;32mComplete to initialize submap!\033[0m";
        LOG_IF(INFO, cfg_.verbose_) << map_arranged_global_->points.size() <<" to " << map_arranged_->points.size() <<" | " <<map_arranged_complement_->points.size();
    } else {
        double diff_x = abs(submap_center_x_ - pose_x);
        double diff_y = abs(submap_center_y_ - pose_y);
        static double half_size = submap_size_ / 2.0;
        if ( (diff_x > half_size) ||  (diff_y > half_size) ) {
            // Reassign submap
            map_arranged_global_.reset(new pcl::PointCloud<pcl::PointXYZI>());
            map_arranged_global_->reserve(num_pcs_init_);
            *map_arranged_global_ = *map_arranged_ + *map_arranged_complement_;

            set_submap(*map_arranged_global_, *map_arranged_, *map_arranged_complement_, pose_x, pose_y, submap_size_);
            submap_center_x_ = pose_x;
            submap_center_y_ = pose_y;
            LOG_IF(INFO, cfg_.verbose_) << "\033[1;32mComplete to initialize submap!\033[0m";
            LOG_IF(INFO, cfg_.verbose_) << map_arranged_global_->points.size() <<" to " << map_arranged_->points.size() <<" | " <<map_arranged_complement_->points.size();
        }
    }
}
void MapUpdater::set_submap(const pcl::PointCloud<pcl::PointXYZI> &map_global, 
                            pcl::PointCloud<pcl::PointXYZI>& submap,
                            pcl::PointCloud<pcl::PointXYZI>& submap_complement,
                            double x, double y, double submap_size) {

    submap.clear();
    submap.reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);
    submap_complement.clear();
    submap_complement.reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);

    for (const auto pt: map_global.points) {
        double diff_x = fabs(x - pt.x);
        double diff_y = fabs(y - pt.y);
        if ((diff_x < submap_size) && (diff_y < submap_size)) {
            submap.points.emplace_back(pt);
        } else {
            submap_complement.points.emplace_back(pt);
        }
    }
}
}  // namespace erasor