/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * MIT License
 * Author: Kin ZHANG (https://kin-zhang.github.io/)
 * Date: 2023-04-04 21:03
 * Description: No ROS version, speed up the process
 *
 * Input: PCD files + Prior raw global map , check our benchmark in dufomap
 * Output: Cleaned global map
 */

#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "MapUpdater.h"
#include "timer.h"

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    FLAGS_colorlogtostderr = true;
    google::SetStderrLogging(google::INFO);

    if (argc != 3) {
        LOG(ERROR) << "Usage: ./erasor_pcd [pcd_folder] [config_file]";
        return 0;
    }
    std::string pcd_parent = argv[1]; // we assume that rawmap is in pcd_parent;
    std::string config_file = argv[2];

    // check if the config_file exists
    if (!std::filesystem::exists(config_file)) {
        LOG(ERROR) << "Config file does not exist: " << config_file;
        return 0;
    }
    erasor::MapUpdater map_updater(config_file);

    // load raw map
    std::string rawmap_path = pcd_parent + "/rawmap.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr rawmap(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile<pcl::PointXYZI>(rawmap_path, *rawmap);
    LOG(INFO) << "Raw map loaded, size: " << rawmap->size();
    map_updater.setRawMap(rawmap);
    // load pcd files
    std::vector<std::string> filenames;
    for (const auto & entry : std::filesystem::directory_iterator(std::filesystem::path(pcd_parent) / "pcd")) {
        filenames.push_back(entry.path().string());
    }
    for (const auto & filename : filenames) {
        // check if the file is pcd
        if (filename.find(".pcd") == std::string::npos) {
            continue;
        }
        // load pcd file
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcd(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::io::loadPCDFile<pcl::PointXYZI>(filename, *pcd);
        // TODO load pcd file in erasor and update static map
        map_updater.run(pcd);
        LOG(INFO) << "Processing: " << filename;
        break; // TODO
    }
    
    // TODO: save static map from ERASOR
    map_updater.saveMap(pcd_parent + "/erasor_output.pcd");
    LOG(INFO) << ANSI_GREEN << "Done!";
    return 0;
}