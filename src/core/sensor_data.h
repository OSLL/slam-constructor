#ifndef __SENSOR_DATA_H
#define __SENSOR_DATA_H

#include <vector>
#include "state_data.h"

struct ScanPoint {
  ScanPoint(double rng = 0, double ang = 0, bool is_occ = true):
    range(rng), angle(ang), is_occupied(is_occ) {}

  double range;
  double angle; // radians
  bool is_occupied;
};

struct TransformedLaserScan {
  RobotPoseDelta pose_delta;

  std::vector<ScanPoint> points;
  double quality; // 0 - low, 1 - fine
};

#endif
