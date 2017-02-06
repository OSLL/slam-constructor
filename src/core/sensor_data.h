#ifndef __SENSOR_DATA_H_INCLUDED
#define __SENSOR_DATA_H_INCLUDED

#include <memory>
#include <vector>
#include "state_data.h"
#include "robot_pose.h"
#include "geometry_utils.h"

struct ScanPoint {
  ScanPoint(double rng = 0, double ang = 0, bool is_occ = true):
    range(rng), angle(ang), is_occupied(is_occ) {}

  double range;
  double angle; // radians
  bool is_occupied;
};

struct TransformedLaserScan {
  // TODO: create simple and effective way
  //       to translate LS to world by a given pose
  std::shared_ptr<TrigonometricCache> trig_cache;
  RobotPoseDelta pose_delta;

  std::vector<ScanPoint> points;
  double quality; // 0 - low, 1 - fine
};

struct AreaOccupancyObservation {
  bool is_occupied;
  Occupancy occupancy;
  Point2D obstacle;
  double quality;
};

#endif
