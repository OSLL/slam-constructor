#include <iostream>
#include <memory>
#include <random>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#include "../../ros/topic_with_transform.h"
#include "../../ros/laser_scan_observer.h"
#include "../../ros/init_utils.h"

#include "../../ros/launch_properties_provider.h"
#include "init_viny_slam.h"

using ObservT = sensor_msgs::LaserScan;
using VinySlamMap = VinySlam::MapType;

int main(int argc, char** argv) {
  ros::init(argc, argv, "vinySLAM");

  auto props = LaunchPropertiesProvider{};
  auto slam = init_viny_slam(props);

  // connect the slam to a ros-topic based data provider
  ros::NodeHandle nh;
  double ros_map_publishing_rate, ros_tf_buffer_size;
  int ros_filter_queue, ros_subscr_queue;
  init_constants_for_ros(ros_tf_buffer_size, ros_map_publishing_rate,
                         ros_filter_queue, ros_subscr_queue);
  auto scan_provider = std::make_unique<TopicWithTransform<ObservT>>(
    nh, laser_scan_2D_ros_topic_name(props), tf_odom_frame_id(props),
    ros_tf_buffer_size, ros_filter_queue, ros_subscr_queue
  );
  auto occup_grid_pub_pin = create_occupancy_grid_publisher<VinySlamMap>(
    slam.get(), nh, ros_map_publishing_rate);

  auto pose_pub_pin = create_pose_correction_tf_publisher<ObservT, VinySlamMap>(
    slam.get(), scan_provider.get(), props);
  auto rp_pub_pin = create_robot_pose_tf_publisher<VinySlamMap>(slam.get());

  auto scan_obs = std::make_shared<LaserScanObserver>(
    slam, get_skip_exceeding_lsr(props), get_use_trig_cache(props));
  // NB: pose_pub_pin must be subscribed first
  scan_provider->subscribe(scan_obs);

  ros::spin();
}
