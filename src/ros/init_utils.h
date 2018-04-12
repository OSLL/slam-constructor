#ifndef SLAM_CTOR_ROS_INIT_UTILS_H
#define SLAM_CTOR_ROS_INIT_UTILS_H

#include <string>
#include <memory>

#include <ros/ros.h>

#include "../core/states/world.h"

#include "../utils/properties_providers.h"
#include "topic_with_transform.h"
#include "pose_correction_tf_publisher.h"
#include "robot_pose_observers.h"
#include "occupancy_grid_publisher.h"

// TODO: remove
std::string get_string_param(const std::string &name,
                             const std::string &dflt_value) {
  std::string value;
  ros::param::param<std::string>(name, value, dflt_value);
  return value;
}

std::string laser_scan_2D_ros_topic_name(const PropertiesProvider &props) {
  return props.get_str("in/lscan2D/ros/topic/name", "/base_scan");
}

std::string tf_odom_frame_id(const PropertiesProvider &props) {
  return props.get_str("in/odometry/ros/tf/odom_frame_id", "odom_combined");
}

std::string tf_map_frame_id() { // TODO: obtain from props
  return get_string_param("~ros/tf/map_frame_id", "map");
}

std::string tf_robot_pose_frame_id() {
  return get_string_param("~ros/tf/robot_pose_frame_id", "robot_pose");
}

bool is_async_correction() {
  bool async_correction;
  ros::param::param<bool>("~ros/tf/async_correction", async_correction, false);
  return async_correction;
}

// scan handling

bool get_skip_exceeding_lsr(const PropertiesProvider &props) {
  return props.get_bool("in/lscan2D/skip_exceeding_vals",
                        false);
}

// performance

bool get_use_trig_cache(const PropertiesProvider &props) {
  return props.get_bool("slam/performance/use_trig_cache", false);
}

// TODO: move to IO

auto tf_ignored_transforms(const PropertiesProvider &props) {
  std::string Rec_Sep = ":", Entry_Sep = "-";
  std::unordered_map<std::string, std::vector<std::string>> ignores;

  auto data = props.get_str("in/odometry/ros/tf/ignore", "");
  auto next_entry_i = std::string::size_type{0};
  while (next_entry_i < data.length()) {
    auto next_sep_i = data.find(Rec_Sep, next_entry_i);
    if (next_sep_i == std::string::npos) {
      next_sep_i = data.length();
    }

    auto entry = data.substr(next_entry_i, next_sep_i - next_entry_i);
    next_entry_i = next_sep_i + 1;

    auto entry_sep_i = entry.find("-");
    if (entry_sep_i == std::string::npos) {
      std::cout << "[WARN] Unable to parse tf_ignore entry \""
                << entry << "\"" << std::endl;
      continue;
    }
    auto from = entry.substr(0, entry_sep_i);
    auto to = entry.substr(entry_sep_i + 1, entry.size() - entry_sep_i - 1);
    ignores["/" + from].push_back("/" + to);
  }
  return ignores;
}

template <typename ObservT, typename MapT>
std::shared_ptr<PoseCorrectionTfPublisher<ObservT>>
create_pose_correction_tf_publisher(WorldObservable<MapT> *slam,
                                    TopicWithTransform<ObservT> *scan_prov,
                                    const PropertiesProvider &props) {
  auto pose_publisher = std::make_shared<PoseCorrectionTfPublisher<ObservT>>(
    tf_map_frame_id(), tf_odom_frame_id(props), is_async_correction()
  );
  scan_prov->subscribe(pose_publisher);
  slam->subscribe_pose(pose_publisher);
  return pose_publisher;
}

template <typename MapT>
std::shared_ptr<RobotPoseTfPublisher>
create_robot_pose_tf_publisher(WorldObservable<MapT> *slam) {
  auto pose_publisher = std::make_shared<RobotPoseTfPublisher>(
    tf_map_frame_id(), tf_robot_pose_frame_id());
  slam->subscribe_pose(pose_publisher);
  return pose_publisher;
}

template <typename ObservT, typename MapT>
auto make_pose_correction_observation_stamped_publisher(
  WorldObservable<MapT> *slam,
  TopicWithTransform<ObservT> *scan_provider,
  const PropertiesProvider &props) {
  using PosePubT = ObservationStampedRoboPoseTfPublisher<ObservT>;
  auto stamped_pose_publisher =
    std::make_shared<PosePubT>(tf_map_frame_id(),
                               tf_robot_pose_frame_id());
  scan_provider->subscribe(stamped_pose_publisher);
  slam->subscribe_pose(stamped_pose_publisher);
  return stamped_pose_publisher;
}

template <typename MapT>
std::shared_ptr<OccupancyGridPublisher<MapT>>
create_occupancy_grid_publisher(WorldObservable<MapT> *slam,
                                ros::NodeHandle nh,
                                double ros_map_publishing_rate) {
  auto map_publisher = std::make_shared<OccupancyGridPublisher<MapT>>(
    nh.advertise<nav_msgs::OccupancyGrid>("map", 5),
    tf_map_frame_id(), ros_map_publishing_rate);
  slam->subscribe_map(map_publisher);
  return map_publisher;
}

void init_constants_for_ros(double &ros_tf_buffer_size,
                            double &ros_map_rate,
                            int &ros_filter_queue,
                            int &ros_subscr_queue) {
  ros::param::param<double>("~ros/tf/buffer_duration",ros_tf_buffer_size, 5.0);
  ros::param::param<double>("~ros/rviz/map_publishing_rate", ros_map_rate, 5.0);
  ros::param::param<int>("~ros/filter_queue_size",ros_filter_queue, 1000);
  ros::param::param<int>("~ros/subscribers_queue_size",ros_subscr_queue, 1000);
}

#endif
