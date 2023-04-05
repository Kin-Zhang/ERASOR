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


using PointT = pcl::PointXYZI;
// using PointT = pcl::PointXYZRGB;

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    FLAGS_colorlogtostderr = true;
    google::SetStderrLogging(google::INFO);

    if (argc < 3) {
        LOG(ERROR) << "Usage: ./erasor_pcd [pcd_folder] [config_file]";
        return 0;
    }
    std::string pcd_parent = argv[1]; // we assume that rawmap is in pcd_parent;
    std::string config_file = argv[2];
    int cnt = 0, run_max = 1 ;
    // check if the config_file exists
    if (!std::filesystem::exists(config_file)) {
        LOG(ERROR) << "Config file does not exist: " << config_file;
        return 0;
    }
    erasor::MapUpdater<PointT> map_updater(config_file);

    // load raw map
    std::string rawmap_path = pcd_parent + "/raw_map.pcd";
    pcl::PointCloud<PointT>::Ptr rawmap(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile<PointT>(rawmap_path, *rawmap);
    LOG(INFO) << "Raw map loaded, size: " << rawmap->size();
    map_updater.setRawMap(rawmap);

    std::vector<std::string> filenames;
    for (const auto & entry : std::filesystem::directory_iterator(std::filesystem::path(pcd_parent) / "pcd")) {
        filenames.push_back(entry.path().string());
    }
    // sort the filenames
    std::sort(filenames.begin(), filenames.end());

    int total = filenames.size();
    if(argc>3){
        run_max = std::stoi(argv[3]);
        if (run_max == 1) {
            LOG(INFO) << "We will run all the frame in sequence, the total number is: " << total;
        }
    }
    TIC;
    for (const auto & filename : filenames) {
        std::ostringstream log_msg;
        log_msg << "(" << cnt << "/" << total << ") Processing: " << filename;
        LOG(INFO) << log_msg.str();

        if (filename.find(".pcd") == std::string::npos)
            continue;

        pcl::PointCloud<PointT>::Ptr pcd(new pcl::PointCloud<PointT>);
        pcl::io::loadPCDFile<PointT>(filename, *pcd);
        map_updater.run(pcd);
        
        cnt++;
        if(cnt>run_max)
            break;
        TOC("erasor run", 1);
    }
    
    map_updater.saveMap(pcd_parent);
    LOG(INFO) << ANSI_GREEN << "Done! " << ANSI_RESET << "Check the output in " << pcd_parent << "/erasor_output.pcd";
    return 0;
}