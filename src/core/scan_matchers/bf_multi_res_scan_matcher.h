#ifndef SLAM_CTOR_CORE_BRUTE_FORCE_MULTIRES_SCAN_MATCHER_H
#define SLAM_CTOR_CORE_BRUTE_FORCE_MULTIRES_SCAN_MATCHER_H

#include <utility>

#include "grid_scan_matcher.h"
#include "m3rsm_engine.h"
#include "../maps/rescalable_caching_grid_map.h"
#include "../geometry_primitives.h"

class BruteForceMultiResolutionScanMatcher : public GridScanMatcher {
private: // consts
  static constexpr double _Max_Translation_Error = 1,
                          _Max_Rotation_Error = deg2rad(5);
public:
  BruteForceMultiResolutionScanMatcher(SPE est,
                                       double ang_step = deg2rad(0.1),
                                       double transl_step = 0.05)
    : GridScanMatcher{est, _Max_Translation_Error, _Max_Translation_Error,
                      _Max_Rotation_Error}
    , _ang_step{ang_step}, _transl_step{transl_step} {}

  void set_target_accuracy(double angle_step, double translation_step) {
    _ang_step = angle_step;
    _transl_step = translation_step;
  }

  double process_scan(const TransformedLaserScan &raw_scan,
                      const RobotPose &pose,
                      const GridMap &map,
                      RobotPoseDelta &result_pose_delta) override {
    do_for_each_observer([&pose, &raw_scan, &map](ObsPtr obs) {
      obs->on_matching_start(pose, raw_scan, map);
    });

    /* setup engine */
    _engine.reset_engine_state();
    _engine.set_translation_lookup_range(max_x_error(), max_y_error());
    // TODO: dynamic angle step estimate
    _engine.set_rotation_lookup_range(2*max_th_error(), _ang_step);

    // FIXME: API - const cast (inside the RAII obj) to be able to do rescaling
    // NB: Do not make 'rescale' const (doing const_cast client probably
    //     won't forget to save/restore current scale)
    SafeRescalableMap rescalable_map{map};
    _engine.add_scan_matching_request(scan_probability_estimator(), pose,
                                      raw_scan.scan, rescalable_map, true);
    while (1) {
      auto best_match = _engine.next_best_match(_transl_step);
      assert(best_match.is_valid());
      if (best_match.is_finest()) {
        result_pose_delta = {best_match.translation_drift.center(),
                             best_match.rotation};
        do_for_each_observer([&result_pose_delta, &best_match](ObsPtr obs) {
            obs->on_matching_end(result_pose_delta, *best_match.filtered_scan,
                                 best_match.prob_upper_bound);
        });
        return best_match.prob_upper_bound;
      }
      // Add exact translation candidates as matches
      // NB: center doesn't guarentee the optimal translation pick
      //     so (as a heuristic) test extra hypotheses in the rectangle.
      auto crucial_points = best_match.translation_drift.corners();
      crucial_points.push_back(best_match.translation_drift.center());

      for (const auto &cp : crucial_points) {
        _engine.add_match(Match{M3RSMEngine::Rect{cp}, best_match});
      }
    }
  }

private:
  M3RSMEngine _engine;
  double _ang_step, _transl_step;
};

#endif // include guard
