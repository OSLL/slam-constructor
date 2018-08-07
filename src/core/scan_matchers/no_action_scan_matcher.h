#ifndef SLAM_CTOR_CORE_NO_ACTION_SCAN_MATCHER_H
#define SLAM_CTOR_CORE_NO_ACTION_SCAN_MATCHER_H

#include <cassert>
#include "grid_scan_matcher.h"

class NoActionScanMatcher : public GridScanMatcher {
public:
  NoActionScanMatcher(SPE estimator) : GridScanMatcher{estimator} {}

  double process_scan(const TransformedLaserScan &scan,
                      const RobotPose &init_pose,
                      const GridMap &map,
                      RobotPoseDelta &pose_delta) override {
    pose_delta = RobotPoseDelta{0, 0, 0.0};
    return 1.0;
  }
};

#endif
