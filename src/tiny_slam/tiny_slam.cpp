#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <memory>

#include <nav_msgs/OccupancyGrid.h>

#include "../ros/topic_with_transform.h"
#include "../ros/pose_correction_tf_publisher.h"
#include "../ros/occupancy_grid_publisher.h"
#include "../ros/laser_scan_observer.h"

#include "../core/sensor_data.h"
#include "../core/maps/area_occupancy_estimator.h"
#include "../core/maps/const_occupancy_estimator.h"
#include "../core/maps/grid_cell_strategy.h"

#include "tiny_world.h"
#include "tiny_grid_cell.h"

std::shared_ptr<GridCell> init_cell_prototype(TinyWorldParams &params) {
  std::string cell_type;
  ros::param::param<std::string>("~cell_type", cell_type, "avg");

  if (cell_type == "base") {
    params.localized_scan_quality = 0.2;
    params.raw_scan_quality = 0.1;
    return std::make_shared<BaseTinyCell>();
  } else if (cell_type == "avg") {
    params.localized_scan_quality = 0.9;
    params.raw_scan_quality = 0.6;
    return std::make_shared<AvgTinyCell>();
  } else {
    std::cerr << "Unknown cell type: " << cell_type << std::endl;
    std::exit(-1);
  }
}

std::shared_ptr<CellOccupancyEstimator> init_occ_estimator() {
  double occ_prob, empty_prob;
  ros::param::param<double>("~base_occupied_prob", occ_prob, 0.95);
  ros::param::param<double>("~base_empty_prob", empty_prob, 0.01);

  using OccEstPtr = std::shared_ptr<CellOccupancyEstimator>;
  std::string est_type;
  ros::param::param<std::string>("~occupancy_estimator", est_type, "const");

  if (est_type == "const") {
    return OccEstPtr{new ConstOccupancyEstimator(occ_prob, empty_prob)};
  } else if (est_type == "area") {
    return OccEstPtr{new AreaOccupancyEstimator(occ_prob, empty_prob)};
  } else {
    std::cerr << "Unknown estimator type: " << est_type << std::endl;
    std::exit(-1);
  }
}

bool init_skip_exceeding_lsr() {
  bool param_value;
  ros::param::param<bool>("~skip_exceeding_lsr_vals", param_value, false);
  return param_value;
}

TinyWorldParams init_common_world_params() {
  double sig_XY, sig_T, width;
  int lim_bad, lim_totl;
  ros::param::param<double>("~scmtch_sigma_XY_MonteCarlo", sig_XY, 0.2);
  ros::param::param<double>("~scmtch_sigma_theta_MonteCarlo", sig_T, 0.1);
  ros::param::param<int>("~scmtch_limit_of_bad_attempts", lim_bad, 20);
  ros::param::param<int>("~scmtch_limit_of_total_attempts", lim_totl, 100);
  ros::param::param<double>("~hole_width", width,1.5);

  return TinyWorldParams(sig_XY, sig_T, lim_bad, lim_totl, width);
}

GridMapParams init_grid_map_params() {
  GridMapParams params;
  ros::param::param<int>("~map_height_in_meters", params.height, 1000);
  ros::param::param<int>("~map_width_in_meters", params.width, 1000);
  ros::param::param<double>("~map_meters_per_cell", params.meters_per_cell,
                                                                         0.1);
  return params;
}

using ObservT = sensor_msgs::LaserScan;
using TinySlamMap = TinyWorld::MapType;

int main(int argc, char** argv) {
  ros::init(argc, argv, "tinySLAM");

  // init tiny slam
  TinyWorldParams params = init_common_world_params();
  GridMapParams map_params = init_grid_map_params();
  auto cost_est = std::make_shared<TinyScanCostEstimator>();
  auto gcs = std::make_shared<GridCellStrategy>(
    init_cell_prototype(params), cost_est, init_occ_estimator());
  auto slam = std::make_shared<TinyWorld>(gcs, params, map_params);

  // connect the slam to a ros-topic based data provider
  ros::NodeHandle nh;
  TopicWithTransform<ObservT> scan_provider(nh, "laser_scan", "odom_combined");
  auto scan_obs = std::make_shared<LaserScanObserver>(slam,
    init_skip_exceeding_lsr());
  scan_provider.subscribe(scan_obs);

  auto map_publisher = std::make_shared<OccupancyGridPublisher<TinySlamMap>>(
    nh.advertise<nav_msgs::OccupancyGrid>("/map", 5));
  slam->subscribe_map(map_publisher);

  auto pose_publisher = std::make_shared<PoseCorrectionTfPublisher<ObservT>>();
  scan_provider.subscribe(pose_publisher);
  slam->subscribe_pose(pose_publisher);

  ros::spin();
}
