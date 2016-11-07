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
#include "tiny_fascade.h"
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

using ObservT = sensor_msgs::LaserScan;
using TinySlamMap = TinySlamFascade::MapType;

int main(int argc, char** argv) {
  ros::init(argc, argv, "tinySLAM");

  // init tiny slam
  TinyWorldParams params;
  std::shared_ptr<ScanCostEstimator> cost_est{new TinyScanCostEstimator()};
  std::shared_ptr<GridCellStrategy> gcs{new GridCellStrategy{
    init_cell_prototype(params), cost_est, init_occ_estimator()}};
  std::shared_ptr<TinySlamFascade> slam{new TinySlamFascade(gcs,
    params)};

  // connect the slam to a ros-topic based data provider
  ros::NodeHandle nh;
  TopicWithTransform<ObservT> scan_provider(nh, "laser_scan", "odom_combined");
  std::shared_ptr<LaserScanObserver> scan_obs{
    new LaserScanObserver{slam, init_skip_exceeding_lsr()}};
  scan_provider.subscribe(scan_obs);

  auto map_publisher = std::make_shared<OccupancyGridPublisher<TinySlamMap>>(
    nh.advertise<nav_msgs::OccupancyGrid>("/map", 5));
  slam->subscribe_map(map_publisher);

  auto pose_publisher = std::make_shared<PoseCorrectionTfPublisher<ObservT>>();
  scan_provider.subscribe(pose_publisher);
  slam->subscribe_pose(pose_publisher);

  ros::spin();
}
