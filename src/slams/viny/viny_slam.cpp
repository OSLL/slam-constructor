#include <iostream>
#include <memory>
#include <random>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#include "../../ros/topic_with_transform.h"
#include "../../ros/laser_scan_observer.h"
#include "../../ros/init_utils.h"

#include "../../core/maps/plain_grid_map.h"
#include "../../core/scan_matchers/monte_carlo_scan_matcher.h"
#include "../../core/states/single_state_hypothesis_laser_scan_grid_world.h"

#include "viny_scan_probability_estimator.h"
#include "viny_grid_cell.h"

void setup_cell_prototype(SingleStateHypothesisLSGWProperties &props) {
  // FIXME: move to params
  props.localized_scan_quality = 0.9;
  props.raw_scan_quality = 0.6;
  props.cell_prototype = std::make_shared<VinyDSCell>();
}

// FIXME: code duplication (tinySLAM)
double init_hole_width() {
  double hole_width;
  ros::param::param<double>("~vinySlam/hole_width", hole_width, 0.5);
  return hole_width;
}

using ObservT = sensor_msgs::LaserScan;
using VinySlam= SingleStateHypothesisLaserScanGridWorld<UnboundedPlainGridMap>;
using VinySlamMap = VinySlam::MapType;

int main(int argc, char** argv) {
  ros::init(argc, argv, "vinySLAM");

  // init viny slam
  auto slam_props = SingleStateHypothesisLSGWProperties{};
  setup_cell_prototype(slam_props);
  slam_props.gsm = init_monte_carlo_scan_matcher(
    std::make_shared<VinyScanProbabilityEstimator>(init_oope())
  );
  slam_props.gmsa = std::make_shared<WallDistanceBlurringScanAdder>(
    init_occ_estimator(), init_hole_width()
  );
  slam_props.map_props = init_grid_map_params();
  auto slam = std::make_shared<VinySlam>(slam_props);

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
