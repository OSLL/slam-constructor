#ifndef SLAM_CTOR_ROS_LASER_SCAN_OBSERVER_H_INCLUDED
#define SLAM_CTOR_ROS_LASER_SCAN_OBSERVER_H_INCLUDED

#include <utility>
#include <memory>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>
#include <cassert>
#include <iostream>

#include "../core/state_data.h"
#include "../core/robot_pose.h"
#include "../core/world.h"
#include "../core/sensor_data.h"
#include "topic_with_transform.h"

class LaserScanObserver : public TopicObserver<sensor_msgs::LaserScan> {
  using ScanPtr = boost::shared_ptr<sensor_msgs::LaserScan>;
  using DstPtr = std::shared_ptr<SensorDataObserver<TransformedLaserScan>>;
public: //methods

  LaserScanObserver(DstPtr slam, bool skip_max_vals = false):
    _slam(slam), _skip_max_vals(skip_max_vals),
    _cache(std::make_shared<TrigonometricCache>()) {}

  virtual void handle_transformed_msg(
    const ScanPtr msg, const tf::StampedTransform& t) {

    RobotPose new_pose(t.getOrigin().getX(), t.getOrigin().getY(),
                       tf::getYaw(t.getRotation()));

    TransformedLaserScan transformed_scan;
    transformed_scan.scan.points().reserve(msg->ranges.size());
    transformed_scan.quality = 1.0;
    transformed_scan.scan.trig_cache = _cache;
    double a_max = msg->angle_min + msg->angle_increment * msg->ranges.size();
    _cache->update(msg->angle_min, a_max, msg->angle_increment);

    double sp_angle = msg->angle_min;
    for (const auto &range : msg->ranges) {
      bool sp_is_occupied = true;
      double sp_range = range;
      sp_angle += msg->angle_increment;

      if (sp_range < msg->range_min) {
        continue;
      } else if (msg->range_max <= sp_range) {
        sp_is_occupied = false;
        sp_range = msg->range_max;
        if (_skip_max_vals) {
          continue;
        }
      }
      transformed_scan.scan.points().emplace_back(sp_range, sp_angle,
                                                  sp_is_occupied);
    }
    assert(are_equal(sp_angle, a_max));

    transformed_scan.pose_delta = new_pose - _prev_pose;
    _prev_pose = new_pose;

    _slam->handle_sensor_data(transformed_scan);
  }

  const RobotPose &odometry_pose() const { return _prev_pose; }
  void set_odometry_pose(const RobotPose& pose) { _prev_pose = pose; }

private: // fields
  DstPtr _slam;
  bool _skip_max_vals;
  RobotPose _prev_pose;
  std::shared_ptr<TrigonometricCache> _cache;
};

#endif
