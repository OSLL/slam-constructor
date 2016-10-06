#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <memory>

#include <nav_msgs/OccupancyGrid.h>

//#define RVIZ_DEBUG 1

#include "../ros/topic_with_transform.h"
#include "../ros/rviz_grid_viewer.h"
#include "../ros/utils.h"
#include "../ros/laser_scan_observer.h"

#include "../core/sensor_data.h"
#include "../core/maps/area_occupancy_estimator.h"
#include "../core/maps/const_occupancy_estimator.h"
#include "../core/maps/grid_cell_strategy.h"
#include "tiny_fascade.h"
#include "tiny_world.h"
#include "tiny_grid_cells.h"

class PoseScanMatcherObserver : public GridScanMatcherObserver {
public:
  virtual void on_scan_test(const RobotPose &pose,
                            const TransformedLaserScan &scan,
                            double score) override {
    publish_transform("sm_curr_pose", pose);
  }
  virtual void on_pose_update(const RobotPose &pose,
                              const TransformedLaserScan &scan,
                              double score) override {
    publish_transform("sm_best_pose", pose);
  }
private:
    void publish_transform(const std::string& frame_id, const RobotPose& p) {
      publish_2D_transform(frame_id, "odom_combined", p);
    }
};

std::shared_ptr<GridCellFactory> init_cell_factory(TinyWorldParams &params) {
  std::string cell_type;
  ros::param::param<std::string>("~cell_type", cell_type, "avg");

  if (cell_type == "base") {
    params.localized_scan_quality = 0.2;
    params.raw_scan_quality = 0.1;
    return std::make_shared<PlainGridCellFactory<BaseTinyCell>>();
  } else if (cell_type == "avg") {
    params.localized_scan_quality = 0.9;
    params.raw_scan_quality = 0.6;
    return std::make_shared<PlainGridCellFactory<AvgTinyCell>>();
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

int main(int argc, char** argv) {
  ros::init(argc, argv, "tinySLAM");

  // init tiny slam
  TinyWorldParams params;
  std::shared_ptr<ScanCostEstimator> cost_est{new TinyScanCostEstimator()};
  std::shared_ptr<GridCellStrategy> gcs{new GridCellStrategy{
    init_cell_factory(params), cost_est, init_occ_estimator()}};
  std::shared_ptr<TinySlamFascade> slam{new TinySlamFascade(gcs,
    params)};

  // connect the slam to a ros-topic based data provider
  ros::NodeHandle nh;
  TopicWithTransform<sensor_msgs::LaserScan> scan_provider(nh,
      "laser_scan", "odom_combined");
  std::shared_ptr<LaserScanObserver> scan_obs{
    new LaserScanObserver{slam, init_skip_exceeding_lsr()}};
  scan_provider.subscribe(scan_obs);

  std::shared_ptr<RvizGridViewer> viewer(
    new RvizGridViewer(nh.advertise<nav_msgs::OccupancyGrid>("/map", 5)));
  slam->subscribe(viewer);

#ifdef RVIZ_DEBUG
  std::shared_ptr<PoseScanMatcherObserver> obs(new PoseScanMatcherObserver);
  slam->add_scan_matcher_observer(obs);
#endif
  ros::spin();
}
