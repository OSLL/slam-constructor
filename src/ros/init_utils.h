#ifndef __INIT_UTILS_H_INCLUDED
#define __INIT_UTILS_H_INCLUDED

#include <string>
#include <memory>

#include <ros/ros.h>

#include "../core/world.h"
#include "../core/maps/area_occupancy_estimator.h"
#include "../core/maps/const_occupancy_estimator.h"

#include "topic_with_transform.h"
#include "pose_correction_tf_publisher.h"
#include "robot_pose_tf_publisher.h"
#include "occupancy_grid_publisher.h"

std::string get_string_param(const std::string &name,
                             const std::string &dflt_value) {
  std::string value;
  ros::param::param<std::string>(name, value, dflt_value);
  return value;
}

std::string tf_odom_frame_id() {
  return get_string_param("~ros/tf/odom_frame_id", "odom_combined");
}

std::string tf_map_frame_id() {
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

bool init_skip_exceeding_lsr() {
  bool param_value;
  ros::param::param<bool>("~ros/skip_exceeding_lsr_vals", param_value, false);
  return param_value;
}

std::shared_ptr<CellOccupancyEstimator> init_occ_estimator() {
  double occ_prob, occ_qual, empty_prob, empty_qual;
  ros::param::param<double>("~slam/occupancy_estimator/"        \
                            "base_occupied/prob", occ_prob, 0.95);
  ros::param::param<double>("~slam/occupancy_estimator/"        \
                            "base_occupied/qual", occ_qual, 1.0);
  ros::param::param<double>("~slam/occupancy_estimator/"        \
                            "base_empty/prob", empty_prob, 0.01);
  ros::param::param<double>("~slam/occupancy_estimator/"        \
                            "base_empty/qual", empty_qual, 1.0);

  std::string est_type;
  ros::param::param<std::string>("~slam/occupancy_estimator/type",
                                 est_type, "const");

  auto base_occ = Occupancy{occ_prob, occ_qual};
  auto base_empty = Occupancy{empty_prob, empty_qual};
  if (est_type == "const") {
    return std::make_shared<ConstOccupancyEstimator>(base_occ, base_empty);
  } else if (est_type == "area") {
    return std::make_shared<AreaOccupancyEstimator>(base_occ, base_empty);
  } else {
    std::cerr << "Unknown estimator type: " << est_type << std::endl;
    std::exit(-1);
  }
}

template <typename ObservT, typename MapT>
std::shared_ptr<PoseCorrectionTfPublisher<ObservT>>
create_pose_correction_tf_publisher(WorldObservable<MapT> *slam,
                                    TopicWithTransform<ObservT> *scan_prov) {
  auto pose_publisher = std::make_shared<PoseCorrectionTfPublisher<ObservT>>(
    tf_map_frame_id(), tf_odom_frame_id(), is_async_correction()
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

GridMapParams init_grid_map_params() {
  double h, w, scale;
  ros::param::param<double>("~slam/map/height_in_meters", h, 10);
  ros::param::param<double>("~slam/map/width_in_meters", w, 10);
  ros::param::param<double>("~slam/map/meters_per_cell", scale, 0.1);
  return {(int)std::ceil(w / scale), (int)std::ceil(h / scale), scale};
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
