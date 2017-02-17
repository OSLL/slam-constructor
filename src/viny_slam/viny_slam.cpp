#include <iostream>
#include <memory>
#include <random>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#include "../ros/topic_with_transform.h"
#include "../ros/laser_scan_observer.h"
#include "../ros/init_utils.h"

#include "../core/sensor_data.h"
#include "../core/maps/grid_cell_strategy.h"

#include "viny_world.h"
#include "viny_grid_cell.h"

std::shared_ptr<GridCell> init_cell_prototype(VinyWorldParams &params) {
  params.localized_scan_quality = 0.9;
  params.raw_scan_quality = 0.6;
  return std::make_shared<VinyDSCell>();
}

VinyWorldParams init_common_world_params() {
  double sig_XY, sig_T, width;
  int lim_bad, lim_totl, seed;
  ros::param::param<double>("~slam/scmtch/MC/sigma_XY", sig_XY, 0.2);
  ros::param::param<double>("~slam/scmtch/MC/sigma_theta", sig_T, 0.1);
  ros::param::param<int>("~slam/scmtch/MC/limit_of_bad_attempts",
                         lim_bad, 20);
  ros::param::param<int>("~slam/scmtch/MC/limit_of_total_attempts",
                         lim_totl, 100);
  ros::param::param<int>("~slam/scmtch/MC/seed", seed,
                         std::random_device{}());
  ros::param::param<double>("~vinySlam/hole_width", width, 0.5);

  ROS_INFO("MC Scan Matcher seed: %u\n", seed);
  auto sm_params = VinySMParams{sig_XY, sig_T,
                                (unsigned)lim_bad, (unsigned)lim_totl,
                                (unsigned) seed};
  return VinyWorldParams{sm_params, width};
}

using ObservT = sensor_msgs::LaserScan;
using VinySlamMap = VinyWorld::MapType;

int main(int argc, char** argv) {
  ros::init(argc, argv, "vinySLAM");

  // init viny slam
  VinyWorldParams params = init_common_world_params();
  GridMapParams map_params = init_grid_map_params();
  auto cost_est = std::make_shared<VinyScanCostEstimator>();
  auto gcs = std::make_shared<GridCellStrategy>(
    init_cell_prototype(params), cost_est, init_occ_estimator());
  auto slam = std::make_shared<VinyWorld>(gcs, params, map_params);

  // connect the slam to a ros-topic based data provider
  ros::NodeHandle nh;
  double ros_map_publishing_rate, ros_tf_buffer_size;
  int ros_filter_queue, ros_subscr_queue;
  init_constants_for_ros(ros_tf_buffer_size, ros_map_publishing_rate,
                         ros_filter_queue, ros_subscr_queue);
  auto scan_provider = std::make_unique<TopicWithTransform<ObservT>>(
    nh, "laser_scan", tf_odom_frame_id(), ros_tf_buffer_size,
    ros_filter_queue, ros_subscr_queue
  );
  auto scan_obs = std::make_shared<LaserScanObserver>(
    slam, init_skip_exceeding_lsr());
  scan_provider->subscribe(scan_obs);

  auto occup_grid_pub_pin = create_occupancy_grid_publisher<VinySlamMap>(
    slam.get(), nh, ros_map_publishing_rate);

  auto pose_pub_pin = create_pose_correction_tf_publisher<ObservT, VinySlamMap>(
    slam.get(), scan_provider.get());
  auto rp_pub_pin = create_robot_pose_tf_publisher<VinySlamMap>(slam.get());

  ros::spin();
}
