#ifndef __LASER_SCAN_OBSERVER_H
#define __LASER_SCAN_OBSERVER_H

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

    TransformedLaserScan laser_scan;
    laser_scan.points.reserve(msg->ranges.size());
    laser_scan.quality = 1.0;
    laser_scan.trig_cache = _cache;
    double a_max = msg->angle_min + msg->angle_increment * msg->ranges.size();
    _cache->update(msg->angle_min, a_max, msg->angle_increment);
    double angle = msg->angle_min;

    for (const auto &range : msg->ranges) {
      ScanPoint sp(range, angle);
      angle += msg->angle_increment;

      if (sp.range < msg->range_min) {
        continue;
      } else if (msg->range_max <= sp.range) {
        sp.is_occupied = false;
        sp.range = msg->range_max;
        if (_skip_max_vals) {
          continue;
        }
      }
      laser_scan.points.push_back(sp);
    }
    assert(are_equal(angle, a_max));

    laser_scan.pose_delta = new_pose - _prev_pose;
    _prev_pose = new_pose;

    _slam->handle_sensor_data(laser_scan);
  }

private: // fields
  DstPtr _slam;
  bool _skip_max_vals;
  RobotPose _prev_pose;
  std::shared_ptr<TrigonometricCache> _cache;
};

#endif
