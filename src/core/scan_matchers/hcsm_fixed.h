#ifndef SLAM_CTOR_CORE_HCSM_FIXED_H_INCLUDED
#define SLAM_CTOR_CORE_HCSM_FIXED_H_INCLUDED

#include <math.h>
#include <limits>

#include "grid_scan_matcher.h"
#include "../maps/grid_map.h"

// FIXME: setup params
class HillClimbingSMFixed : public GridScanMatcher {
public:
  HillClimbingSMFixed(SPE est,
                      int max_shrinks_nm = 5,
                      double lin_delta = 0.1, double ang_delta = 0.1)
    : GridScanMatcher(est), _angular_delta(ang_delta), _linear_delta(lin_delta)
    , _max_step_shrinks(max_shrinks_nm) {}

  double process_scan(const TransformedLaserScan &raw_scan,
                      const RobotPose &init_pose,
                      const GridMap &map,
                      RobotPoseDelta &pose_delta) override {
    auto scan = filter_scan(raw_scan.scan, init_pose, map);

    const double d_t = 0.01;
    const double d_o = deg2rad(0.5);

    // TODO: store pose as sampled value
    RobotPose best_pose = init_pose;
    double best_pose_prob = scan_probability(scan, best_pose, map);
    for (unsigned step_shrinks = 0; step_shrinks < _max_step_shrinks;
         _max_step_shrinks++) {
      RobotPose current_pose = best_pose;

      // determine direction
      size_t next_action = 0;
      double d_d = 0;
      double best_diff = 1;
      for (size_t action = 0; action < DirNm * DimNm; ++action) {
        RobotPose shifted_pose = shift(current_pose, d_o, d_t, action);
        double shifted_pose_prob = scan_probability(scan, shifted_pose, map);
        double prob_diff = best_pose_prob - shifted_pose_prob;

        if (prob_diff < best_diff) {
          best_diff = prob_diff;
          next_action = action;
          d_d = action % DimNm == Th ? d_o : d_t;
        }
      }

      // determine best value
      double r = 16 * d_d;
      while (d_d < r) {
        auto r_pose = shift(current_pose, r, r, next_action);
        auto r_prob = scan_probability(scan, r_pose, map);
        if (best_pose_prob < r_prob) {
          best_pose_prob = std::move(r_prob);
          best_pose = r_pose;
          break;
        }
        r /= 2;
      }

      if (r == d_d) {
        if (0 <= best_diff) { break; }
        best_pose = shift(current_pose, r, r, next_action);
        best_pose_prob -= best_diff;
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
    switch (action % DimNm) {
    case X:  shifted.x     += shift_value * l_step; break;
    case Y:  shifted.y     += shift_value * l_step; break;
    case Th: shifted.theta += shift_value * a_step; break;
    default: assert(0 && "Missed shift destination");
    }

    return shifted;
  }

private:
  double _angular_delta, _linear_delta;
  unsigned _max_step_shrinks;
};

#endif
