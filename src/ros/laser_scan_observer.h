#ifndef SLAM_CTOR_ROS_LASER_SCAN_OBSERVER_H_INCLUDED
#define SLAM_CTOR_ROS_LASER_SCAN_OBSERVER_H_INCLUDED

#include <utility>
#include <memory>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>
#include <cassert>
#include <iostream>

#include "../core/states/state_data.h"
#include "../core/states/robot_pose.h"
#include "../core/states/world.h"
#include "../core/states/sensor_data.h"
#include "topic_with_transform.h"

class LaserScanObserver : public TopicObserver<sensor_msgs::LaserScan> {
  using ScanPtr = boost::shared_ptr<sensor_msgs::LaserScan>;
  using DstPtr = std::shared_ptr<SensorDataObserver<TransformedLaserScan>>;
public: //methods

  LaserScanObserver(DstPtr slam,
                    bool skip_max_vals,
                    bool use_cached_trig)
    : _slam(slam), _skip_max_vals(skip_max_vals)
    , _use_cached_trig_provider{use_cached_trig} {}

  void handle_transformed_msg(const ScanPtr msg,
                              const tf::StampedTransform& t) override {
    RobotPose new_pose(t.getOrigin().getX(), t.getOrigin().getY(),
                       tf::getYaw(t.getRotation()));

    auto transformed_scan = msg2scan(msg);
    transformed_scan.pose_delta = new_pose - _prev_pose;
    _prev_pose = new_pose;

    _slam->handle_sensor_data(transformed_scan);
  }

  TransformedLaserScan msg2scan(const ScanPtr msg) const {
    auto transformed_scan = TransformedLaserScan{};
    transformed_scan.scan.points().reserve(msg->ranges.size());
    transformed_scan.quality = 1.0;
    // TODO: move trig provider setup to the SLAM
    transformed_scan.scan.trig_provider = trig_provider(msg);

    double sp_angle = msg->angle_min - msg->angle_increment;
    double range_max = -1.0;
    double range_min = msg->range_max + 1.0;
    for (const auto &range : msg->ranges) {
      bool sp_is_occupied = true;
      double sp_range = range;
      sp_angle += msg->angle_increment;

      // filter points by range/angle
      if (sp_range < msg->range_min) {
        continue;
      } else if (msg->range_max <= sp_range) {
        sp_is_occupied = false;
        sp_range = msg->range_max;
        if (_skip_max_vals) {
          continue;
        }
      }


      if (range_max < sp_range)
        range_max = sp_range;
      if(sp_range < range_min)
        range_min = sp_range;
      
      // add a scan point to a scan
      transformed_scan.scan.points().emplace_back(sp_range, sp_angle,
                                                  sp_is_occupied);
    }
    transformed_scan.range_max = range_max;
    transformed_scan.range_min = range_min;
    assert(are_equal<double>(sp_angle, msg->angle_max, deg2rad(0.001)));
    return transformed_scan;
  }

  const RobotPose &odometry_pose() const { return _prev_pose; }
  void set_odometry_pose(const RobotPose& pose) { _prev_pose = pose; }

private:

  std::shared_ptr<TrigonometryProvider> trig_provider(const ScanPtr msg) const {
    if (_use_cached_trig_provider) {
      auto provider = std::make_shared<CachedTrigonometryProvider>();
      provider->update(msg->angle_min, msg->angle_max + msg->angle_increment,
                       msg->angle_increment);
      return provider;
    } else {
      return std::make_shared<RawTrigonometryProvider>();
    }
  }

private: // fields
  DstPtr _slam;
  bool _skip_max_vals;
  bool _use_cached_trig_provider;
  RobotPose _prev_pose;
};

#endif
