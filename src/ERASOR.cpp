#include "ERASOR.h"

#include <glog/logging.h>
#include <pcl/common/centroid.h>

#include "timer.h"
namespace benchmark {

ERASOR::ERASOR() {
}

void ERASOR::clear_bin(Bin &bin) {
  bin.max_h = -INF;
  bin.min_h = INF;
  bin.x = 0;
  bin.y = 0;
  bin.is_occupied = false;
  bin.status = NOT_ASSIGNED;
  if (!bin.points.empty()) bin.points.clear();
}

void ERASOR::clear(pcl::PointCloud<pcl::PointXYZI> &pt_cloud) {
  if (!pt_cloud.empty()) {
    pt_cloud.clear();
  }
}
/**
 * @brief Inputs should be the transformed pointcloud!
 */
void ERASOR::set_inputs(const pcl::PointCloud<pcl::PointXYZI> &map_voi,
                        const pcl::PointCloud<pcl::PointXYZI> &query_voi) {
  TIC;
  clear(map_complement);
  for (int theta = 0; theta < cfg_.num_sectors_; ++theta) {
    for (int r = 0; r < cfg_.num_rings_; ++r) {
      clear_bin(r_pod_map[r][theta]);
      clear_bin(r_pod_curr[r][theta]);
      clear_bin(r_pod_selected[r][theta]);
    }
  }

  voi2r_pod(query_voi, r_pod_curr);
  voi2r_pod(map_voi, r_pod_map, map_complement);
  int debug_total_num = 0;
  for (int theta = 0; theta < cfg_.num_sectors_; theta++) {
    for (int r = 0; r < cfg_.num_rings_; r++) {
      Bin &bin_map = r_pod_map[r][theta];
      debug_total_num += bin_map.points.size();
    }
  }
}
void ERASOR::setConfig(common::Config &cfg) {
  cfg_ = cfg;
  LOG(INFO) << "Set Config for ERASOR";

  // Initalization of R-POD
  init(r_pod_map);
  init(r_pod_curr);
  init(r_pod_selected);

  ring_size = cfg_.max_range_ / cfg_.num_rings_;
  sector_size = 2 * PI / cfg_.num_sectors_;

  piecewise_ground_.reserve(130000);
  non_ground_.reserve(130000);
  ground_pc_.reserve(130000);
  non_ground_pc_.reserve(130000);
  LOG(INFO) << "ERASOR is initialized";
}

void ERASOR::pt2r_pod(const pcl::PointXYZI &pt, Bin &bin) {
  bin.is_occupied = true;
  bin.points.push_back(pt);
  if (pt.z >= bin.max_h) {
    bin.max_h = pt.z;
    bin.x = pt.x;
    bin.y = pt.y;
  }
  if (pt.z <= bin.min_h) {
    bin.min_h = pt.z;
  }
}

double ERASOR::xy2theta(const double &x, const double &y) {  // 0 ~ 2 * PI
  if (y >= 0) {
    return atan2(y, x);  // 1, 2 quadrant
  } else {
    return 2 * PI + atan2(y, x);  // 3, 4 quadrant
  }
}

double ERASOR::xy2radius(const double &x, const double &y) {
  return sqrt(pow(x, 2) + pow(y, 2));
}

void ERASOR::voi2r_pod(const pcl::PointCloud<pcl::PointXYZI> &src,
                       R_POD &r_pod) {
  for (auto const &pt : src.points) {
    if (pt.z < cfg_.max_h_ && pt.z > cfg_.min_h_) {
      double r = xy2radius(pt.x, pt.y);
      if (r <= cfg_.max_range_) {
        double theta = xy2theta(pt.x, pt.y);

        int sector_idx =
            min(static_cast<int>((theta / sector_size)), cfg_.num_sectors_ - 1);
        int ring_idx =
            min(static_cast<int>((r / ring_size)), cfg_.num_rings_ - 1);

        pt2r_pod(pt, r_pod.at(ring_idx).at(sector_idx));
      }
    }
  }

  // For debugging
  // pcl::PointCloud<pcl::PointXYZI> curr_init;
  // r_pod2pc(r_pod, curr_init);
  // sensor_msgs::PointCloud2 pc2_curr_init =
  // erasor_utils::cloud2msg(curr_init); pub_curr_init.publish(pc2_curr_init);
}

void ERASOR::voi2r_pod(const pcl::PointCloud<pcl::PointXYZI> &src, R_POD &r_pod,
                       pcl::PointCloud<pcl::PointXYZI> &complement) {
  for (auto const &pt : src.points) {
    if (pt.z < cfg_.max_h_ && pt.z > cfg_.min_h_) {  // range of z?
      double r = xy2radius(pt.x, pt.y);
      if (r <= cfg_.max_range_) {
        double theta = xy2theta(pt.x, pt.y);
        //        int sector_idx = min(static_cast<int>((theta / sector_size) +
        //        0.5), num_sectors - 1); int ring_idx = min(static_cast<int>((r
        //        / ring_size) + 0.5), num_rings - 1);
        int sector_idx =
            min(static_cast<int>((theta / sector_size)), cfg_.num_sectors_ - 1);
        int ring_idx =
            min(static_cast<int>((r / ring_size)), cfg_.num_rings_ - 1);
        pt2r_pod(pt, r_pod.at(ring_idx).at(sector_idx));
      } else {
        complement.points.push_back(pt);
      }
    } else {
      complement.points.push_back(pt);
    }
  }
  // pcl::PointCloud<pcl::PointXYZI> map_init;
  // r_pod2pc(r_pod, map_init);
  // sensor_msgs::PointCloud2 pc2_map_init = erasor_utils::cloud2msg(map_init);
  // pub_map_init.publish(pc2_map_init);
}

void ERASOR::compare_vois_and_revert_ground_w_block() {
  
  for (int theta = 0; theta < cfg_.num_sectors_; theta++) {
    for (int r = 0; r < cfg_.num_rings_; r++) {
      // Min. num of pts criteria.
      Bin &bin_curr = r_pod_curr[r][theta];
      Bin &bin_map = r_pod_map[r][theta];

      if (bin_map.points.empty()) {
        r_pod_selected[r][theta].status = LITTLE_NUM;
        continue;
      }
      
      if (bin_curr.points.size() < cfg_.minimum_num_pts) {
        r_pod_selected[r][theta].status = LITTLE_NUM;
      } else {
        double map_h_diff = bin_map.max_h - bin_map.min_h;
        double curr_h_diff = bin_curr.max_h - bin_curr.min_h;
        double scan_ratio =
            min(map_h_diff / curr_h_diff, curr_h_diff / map_h_diff);
        // ---------------------------------
        //          Scan Ratio Test
        // ---------------------------------
        if (bin_curr.is_occupied && bin_map.is_occupied) {
          if (scan_ratio < cfg_.scan_ratio_threshold) {  // find dynamic!
            if (map_h_diff >= curr_h_diff) {  // Occupied -> Disappear  <<BLUE>>
              r_pod_selected[r][theta].status = MAP_IS_HIGHER;
            } else if (map_h_diff <=
                       curr_h_diff) {  // No objects exist -> Appear! <<GREEN>>
              r_pod_selected[r][theta].status = CURR_IS_HIGHER;
            }
          } else {
            r_pod_selected[r][theta].status = MERGE_BINS;
          }
        } else if (bin_map.is_occupied) {  // Maybe redundant?
          r_pod_selected[r][theta].status = LITTLE_NUM;
        }
      }
    }
  }
  
  // 2. set bins!
  for (int theta = 0; theta < cfg_.num_sectors_; theta++) {
    for (int r = 0; r < cfg_.num_rings_; r++) {
      Bin &bin_curr = r_pod_curr[r][theta];
      Bin &bin_map = r_pod_map[r][theta];

      double OCCUPANCY_STATUS = r_pod_selected[r][theta].status;
      if (OCCUPANCY_STATUS == LITTLE_NUM) {
        r_pod_selected[r][theta] = bin_map;
        r_pod_selected[r][theta].status = LITTLE_NUM;
      } else if (OCCUPANCY_STATUS == MAP_IS_HIGHER) {
        if ((bin_map.max_h - bin_map.min_h) > 0.5) {
          r_pod_selected[r][theta] = bin_curr;
          r_pod_selected[r][theta].status = MAP_IS_HIGHER;
          // ---------------------------------
          //     NOTE: Ground is retrieved!
          // ---------------------------------

          if (!piecewise_ground_.empty()) piecewise_ground_.clear();
          if (!non_ground_.empty()) non_ground_.clear();

          extract_ground(bin_map.points, piecewise_ground_, non_ground_);
          /*** It potentially requires lots of memories... */
          r_pod_selected[r][theta].points += piecewise_ground_;

          /*** Thus, voxelization is conducted */
          pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(
              new pcl::PointCloud<pcl::PointXYZI>);
          *tmp = r_pod_selected[r][theta].points;

        } else {
          r_pod_selected[r][theta] = bin_map;
          r_pod_selected[r][theta].status = NOT_ASSIGNED;
        }

      } else if (OCCUPANCY_STATUS == CURR_IS_HIGHER) {
        r_pod_selected[r][theta] = bin_map;
        r_pod_selected[r][theta].status = CURR_IS_HIGHER;

      } else if (OCCUPANCY_STATUS == MERGE_BINS) {
        if (is_dynamic_obj_close(r_pod_selected, r, theta, 1, 1)) {
          r_pod_selected[r][theta] = bin_map;
          r_pod_selected[r][theta].status = BLOCKED;

        } else {
          // NOTE the dynamic object comes ....:(
          r_pod_selected[r][theta] = bin_map;
          r_pod_selected[r][theta].status = MERGE_BINS;
        }
      }
    }
  }
}

void ERASOR::r_pod2pc(const R_POD &sc, pcl::PointCloud<pcl::PointXYZI> &pc) {
    pc.points.clear();
    for (int theta = 0; theta < cfg_.num_sectors_; theta++) {
        for (int r = 0; r < cfg_.num_rings_; r++) {
            if (sc.at(r).at(theta).is_occupied) {
                for (auto const &pt : sc.at(r).at(theta).points) {
                    pc.points.push_back(pt);
                }
            }
        }
    }
}

void ERASOR::get_static_estimate(
        pcl::PointCloud<pcl::PointXYZI> &arranged,
        pcl::PointCloud<pcl::PointXYZI> &complement) {
    r_pod2pc(r_pod_selected, arranged);
    // arranged += ground_viz;
    complement = map_complement;
}

void ERASOR::init(R_POD &r_pod) {
  if (!r_pod.empty()) {
    r_pod.clear();
  }
  Ring ring;
  Bin bin = {-INF, INF, 0, 0, false, static_cast<bool>(NOT_ASSIGNED)};
  bin.points.reserve(ENOUGH_NUM);
  for (int i = 0; i < cfg_.num_sectors_; i++) {
    ring.emplace_back(bin);
  }
  for (int j = 0; j < cfg_.num_rings_; j++) {
    r_pod.emplace_back(ring);
  }
}

bool ERASOR::is_dynamic_obj_close(R_POD &r_pod_selected, int r_target,
                                  int theta_target, int r_range,
                                  int theta_range) {
  // Set thetas
  std::vector<int> theta_candidates;
  for (int j = theta_target - theta_range; j <= theta_target + theta_range;
       j++) {
    if (j < 0) {
      theta_candidates.push_back(j + cfg_.num_rings_);
    } else if (j >= cfg_.num_sectors_) {
      theta_candidates.push_back(j - cfg_.num_rings_);
    } else {
      theta_candidates.push_back(j);
    }
  }
  for (int r = std::max(0, r_target - r_range);
       r <= std::min(r_target + r_range, cfg_.num_rings_ - 1); r++) {
    for (const auto &theta : theta_candidates) {
      if ((r == r_target) && (theta == theta_target)) continue;

      if (r_pod_selected[r][theta].status ==
          CURR_IS_HIGHER) {  // Dynamic object is near
        return true;
      }
    }
  }
  return false;
}

bool point_cmp(pcl::PointXYZI a, pcl::PointXYZI b) { return a.z < b.z; }

void ERASOR::extract_ground(pcl::PointCloud<pcl::PointXYZI> &src,
                            pcl::PointCloud<pcl::PointXYZI> &dst,
                            pcl::PointCloud<pcl::PointXYZI> &outliers) {
  if (!dst.empty()) dst.clear();
  if (!outliers.empty()) outliers.clear();

  auto src_copy = src;
  std::sort(src_copy.points.begin(), src_copy.points.end(), point_cmp);
  // 1. remove_outliers;
  auto it = src_copy.points.begin();
  for (int i = 0; i < src_copy.points.size(); i++) {
    // if(src_copy.points[i].z < -0.5*SENSOR_HEIGHTS){
    if (src_copy.points[i].z < cfg_.min_h_) {
      it++;
    } else {
      break;
    }
  }
  src_copy.points.erase(src_copy.points.begin(), it);

  // 2. set seeds!
  if (!ground_pc_.empty()) ground_pc_.clear();
  if (!non_ground_pc_.empty()) non_ground_pc_.clear();

  extract_initial_seeds_(src_copy, ground_pc_);
  //  std::cout<<"\033[1;032m [Ground] num:
  //  "<<ground_pc.points.size()<<std::endl;
  // 3. Extract ground
  for (int i = 0; i < cfg_.iter_groundfilter_; i++) {
    estimate_plane_(ground_pc_);
    ground_pc_.clear();

    // pointcloud to matrix
    Eigen::MatrixXf points(src.points.size(), 3);
    int j = 0;
    for (auto p : src.points) {
      points.row(j++) << p.x, p.y, p.z;
    }
    // ground plane model
    Eigen::VectorXf result = points * normal_;
    // threshold filter
    for (int r = 0; r < result.rows(); r++) {
      if (result[r] < th_dist_d_) {
        ground_pc_.points.push_back(src[r]);
      } else {
        if (i == (cfg_.iter_groundfilter_ - 1)) {  // Last iteration
          non_ground_pc_.points.push_back(src[r]);
        }
      }
    }
    //      std::cout<<"\033[1;032m [Ground] ith: "<< i<<" | num:
    //      "<<ground_pc.points.size()<<std::endl;
  }
  dst = ground_pc_;
  outliers = non_ground_pc_;
  //  if (normal_(2,0) > 0.707){// perpendicular to ground
  //    dst = ground_pc_;
  //    outliers = non_ground_pc_;
  //  }else{ // reject results. Do not discern
  //    std::cout<<"rejected!"<<std::endl;
  //    dst = ground_pc_ + non_ground_pc_;
  //  }
}

void ERASOR::estimate_plane_(const pcl::PointCloud<pcl::PointXYZI> &ground) {
  Eigen::Matrix3f cov;
  Eigen::Vector4f pc_mean;
  pcl::computeMeanAndCovarianceMatrix(ground, cov, pc_mean);
  // Singular Value Decomposition: SVD
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(
      cov, Eigen::DecompositionOptions::ComputeFullU);
  // use the least singular vector as normal
  normal_ = (svd.matrixU().col(2));
  // mean ground seeds value
  Eigen::Vector3f seeds_mean = pc_mean.head<3>();

  // according to normal.T*[x,y,z] = -d
  d_ = -(normal_.transpose() * seeds_mean)(0, 0);
  // set distance threhold to `th_dist - d`
  th_dist_d_ = cfg_.th_dist_ - d_;
}

void ERASOR::extract_initial_seeds_(
    const pcl::PointCloud<pcl::PointXYZI> &p_sorted,
    pcl::PointCloud<pcl::PointXYZI> &init_seeds) {
  init_seeds.points.clear();
  pcl::PointCloud<pcl::PointXYZI> g_seeds_pc;

  // LPR is the mean of low point representative
  double sum = 0;
  int cnt = 0;

  // Calculate the mean height value.

  for (int i = cfg_.num_lowest_pts;
       i < p_sorted.points.size() && cnt < cfg_.num_lprs_; i++) {
    sum += p_sorted.points[i].z;
    cnt++;
  }
  double lpr_height = cnt != 0 ? sum / cnt : 0;  // in case divide by 0
  g_seeds_pc.clear();
  // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
  for (int i = 0; i < p_sorted.points.size(); i++) {
    if (p_sorted.points[i].z < lpr_height + cfg_.th_seeds_heights_) {
      g_seeds_pc.points.push_back(p_sorted.points[i]);
    }
  }
  //  std::cout<<"hey!! g seeds"<<g_seeds_pc.points.size()<<std::endl;
  // return seeds points
  init_seeds = g_seeds_pc;
}

}  // namespace benchmark