#ifndef __GRADIENT_WALKER_SCAN_MATCHER_H_INCLUDED
#define __GRADIENT_WALKER_SCAN_MATCHER_H_INCLUDED

#include <math.h>
#include <limits>

#include "grid_scan_matcher.h"
#include "maps/grid_map.h"

class GradientWalkerScanMatcher : public GridScanMatcher {
public:
  GradientWalkerScanMatcher(std::shared_ptr<ScanCostEstimator> est) :
     GridScanMatcher(est), _angular_delta(0.1), _linear_delta(0.1),
     _max_step_shrinks(6) {}

    virtual double process_scan(const RobotPose &init_pose,
                                const TransformedLaserScan &scan,
                                const GridMap &map,
                                RobotPoseDelta &pose_delta) override {
      auto sce = GridScanMatcher::cost_estimator();
      double a_step = _angular_delta, l_step = _linear_delta;
      RobotPose best_pose = init_pose;
      double best_prob = sce->estimate_scan_cost(best_pose, scan, map, 1.0);

      for (unsigned step_shrinks = 0; step_shrinks < _max_step_shrinks;) {
        RobotPose current_pose = best_pose;
        bool pose_was_shifted = false;

        for (size_t action = 0; action < DirNm * DimNm; ++action) {
          RobotPose shifted_pose = shift(current_pose, a_step, l_step, action);
          double shifted_pose_prob =
            sce->estimate_scan_cost(shifted_pose, scan, map, 1.0);

          if (best_prob < shifted_pose_prob) {
            best_pose = shifted_pose;
            best_prob = shifted_pose_prob;
            pose_was_shifted = true;
          }
        }

        if (!pose_was_shifted) {
          a_step *= 0.5, l_step *= 0.5;
          ++step_shrinks;
        }
      }
      pose_delta = best_pose - init_pose;
      return best_prob;
    }

private:
   enum Dim {X = 0, Y, Th, DimNm};
   enum Dir {Inc = 0, Dec, DirNm};
   static_assert(DimNm % 2 != DirNm % 2, "");

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
