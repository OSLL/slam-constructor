#ifndef SLAM_CTOR_CORE_POSE_ENUMERATION_SCAN_MATCHER_H
#define SLAM_CTOR_CORE_POSE_ENUMERATION_SCAN_MATCHER_H

#include <functional>
#include <memory>

#include "pose_enumerators.h"
#include "grid_scan_matcher.h"

// TODO: merge the logic with hill climbing scan matcher
//       create free functions that create scan matchers
// TODO: move publish transform to observer
class PoseEnumerationScanMatcher : public GridScanMatcher {
public:
  PoseEnumerationScanMatcher(std::shared_ptr<ScanProbabilityEstimator> spe,
                             std::shared_ptr<PoseEnumerator> pe)
    : GridScanMatcher{spe}, _pose_enumerator{pe} {}

  void set_pose_enumerator(std::shared_ptr<PoseEnumerator> pe) {
    _pose_enumerator = pe;
  }

  auto pose_enumerator() const { return _pose_enumerator; }

  // GridScanMatcher API implementation

  void reset_state() override {
    _pose_enumerator->reset();
  }

  double process_scan(const TransformedLaserScan &raw_scan,
                      const RobotPose &init_pose,
                      const GridMap &map,
                      RobotPoseDelta &pose_delta) override {
    do_for_each_observer([&init_pose, &raw_scan, &map](ObsPtr obs) {
      obs->on_matching_start(init_pose, raw_scan, map);
    });
    auto scan = filter_scan(raw_scan.scan, init_pose, map);
    auto best_pose = init_pose;
    auto best_pose_prob = scan_probability(scan, best_pose, map);

    do_for_each_observer([best_pose, scan, best_pose_prob](ObsPtr obs) {
      obs->on_scan_test(best_pose, scan, best_pose_prob);
      obs->on_pose_update(best_pose, scan, best_pose_prob);
    });

    _pose_enumerator->reset();
    while (_pose_enumerator->has_next()) {
      auto sampled_pose = _pose_enumerator->next(best_pose);
      double sampled_scan_prob = scan_probability(scan, sampled_pose, map);
      do_for_each_observer([&sampled_pose, &scan,
                            &sampled_scan_prob](ObsPtr obs) {
        obs->on_scan_test(sampled_pose, scan, sampled_scan_prob);
      });

      auto pose_is_acceptable = best_pose_prob < sampled_scan_prob;
      _pose_enumerator->feedback(pose_is_acceptable);
      if (!pose_is_acceptable) {
        continue;
      }

      // update pose
      best_pose_prob = sampled_scan_prob;
      best_pose = sampled_pose;

      // notify pose update
      do_for_each_observer([&best_pose, &scan, &best_pose_prob](ObsPtr obs) {
        obs->on_pose_update(best_pose, scan, best_pose_prob);
      });
    }

    pose_delta = best_pose - init_pose;
    do_for_each_observer([&scan, &pose_delta, &best_pose_prob](ObsPtr obs) {
        obs->on_matching_end(pose_delta, scan, best_pose_prob);
    });
    return best_pose_prob;
  }

private:
  std::shared_ptr<PoseEnumerator> _pose_enumerator;

};

#endif
