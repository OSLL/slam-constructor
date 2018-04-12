#include <memory>
#include <cassert>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#include "../../ros/topic_with_transform.h"
#include "../../ros/laser_scan_observer.h"
#include "../../ros/init_utils.h"

#include "../../ros/launch_properties_provider.h"
#include "init_gmapping.h"

using ObservT = sensor_msgs::LaserScan;
using GmappingMap = Gmapping::MapType;

int main(int argc, char** argv) {
  ros::init(argc, argv, "GMapping");

  auto props = LaunchPropertiesProvider{};
  auto slam = init_gmapping(props);

  ros::NodeHandle nh;
  // TODO: code duplication (viny.cpp)
  double ros_map_publishing_rate, ros_tf_buffer_size;
  int ros_filter_queue, ros_subscr_queue;
  init_constants_for_ros(ros_tf_buffer_size, ros_map_publishing_rate,
                         ros_filter_queue, ros_subscr_queue);
  auto scan_provider = std::make_unique<TopicWithTransform<ObservT>>(
    nh, laser_scan_2D_ros_topic_name(props), tf_odom_frame_id(props),
    ros_tf_buffer_size, ros_filter_queue, ros_subscr_queue
  );
  auto occup_grid_pub_pin = create_occupancy_grid_publisher(
    slam.get(), nh, ros_map_publishing_rate);

  auto pose_pub_pin = create_pose_correction_tf_publisher(
    slam.get(), scan_provider.get(), props);
  // publish a "raw" robot pose to be compatible with test service
  // use ObservationStampedPublishing to be compatible with TUM evaluator
  auto rp_pub_pin = make_pose_correction_observation_stamped_publisher(
    slam.get(), scan_provider.get(), props);

  // TODO: setup scan skip policy via param
  auto scan_obs_pin = std::make_shared<LaserScanObserver>(
    slam, get_skip_exceeding_lsr(props), get_use_trig_cache(props));
  // NB: pose_pub_pin must be subscribed first
  scan_provider->subscribe(scan_obs_pin);

  ros::spin();
}
