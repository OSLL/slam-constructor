#ifndef SLAM_CTOR_CORE_HILL_CLIMBING_SCAN_MATCHER_H_INCLUDED
#define SLAM_CTOR_CORE_HILL_CLIMBING_SCAN_MATCHER_H_INCLUDED

#include <math.h>
#include <limits>

#include "grid_scan_matcher.h"
#include "../maps/grid_map.h"

class HillClimbingScanMatcher : public GridScanMatcher {
public:
  HillClimbingScanMatcher(SPE est,
                          int max_shrinks_nm = 6,
                          double lin_delta = 0.1, double ang_delta = 0.1)
    : GridScanMatcher(est), _angular_delta(ang_delta), _linear_delta(lin_delta)
    , _max_step_shrinks(max_shrinks_nm) {}

  double process_scan(const TransformedLaserScan &scan,
                      const RobotPose &init_pose,
                      const GridMap &map,
                      RobotPoseDelta &pose_delta) override {
    double a_step = _angular_delta, l_step = _linear_delta;

    // TODO: store pose as sampled value
    RobotPose best_pose = init_pose;
    double best_pose_prob = scan_probability(scan, best_pose, map);
    for (unsigned step_shrinks = 0; step_shrinks < _max_step_shrinks;) {
      RobotPose current_pose = best_pose;
      bool pose_was_shifted = false;
      for (size_t action = 0; action < DirNm * DimNm; ++action) {
        RobotPose shifted_pose = shift(current_pose, a_step, l_step, action);
        double shifted_pose_prob = scan_probability(scan, shifted_pose, map);

        if (best_pose_prob < shifted_pose_prob) {
          best_pose = shifted_pose;
          best_pose_prob = shifted_pose_prob;
          pose_was_shifted = true;
        }
      }

      if (!pose_was_shifted) {
        a_step *= 0.5, l_step *= 0.5;
        ++step_shrinks;
      }
    }
    pose_delta = best_pose - init_pose;
    return best_pose_prob;
  }

private:
  enum Dim {X = 0, Y, Th, DimNm};
  enum Dir {Inc = 0, Dec, DirNm};
  static_assert(DimNm % 2 != DirNm % 2, "");

  // TODO: return RobotPoseDelta
  RobotPose shift(const RobotPose &current_pose, double a_step, double l_step,
                  size_t  action) {
    RobotPose shifted = current_pose;

    double shift_value = action % DirNm ? -1 : 1;
    double *shiftee;
    switch (action % DimNm) {
    case X:  shiftee = &shifted.x, shift_value *= l_step; break;
    case Y:  shiftee = &shifted.y, shift_value *= l_step; break;
    case Th: shiftee = &shifted.theta, shift_value *= a_step; break;
    default: assert(0 && "Missed shift destination");
    }

    *shiftee += shift_value;
    return shifted;
  }

private:
  double _angular_delta, _linear_delta;
  unsigned _max_step_shrinks;
};

#endif
