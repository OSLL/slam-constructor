#ifndef SLAM_CTOR_CORE_MONTE_CARLO_SCAN_MATCHER_H_INCLUDED
#define SLAM_CTOR_CORE_MONTE_CARLO_SCAN_MATCHER_H_INCLUDED

#include <functional>
#include <limits>
#include <memory>

#include "grid_scan_matcher.h"

// TODO: move publish transform to observer
class MonteCarloScanMatcher : public GridScanMatcher {
public:
  using ObsPtr = std::shared_ptr<GridScanMatcherObserver>;
public:
  MonteCarloScanMatcher(SPE estimator,
                        unsigned failed_iter, unsigned max_iter):
    GridScanMatcher{estimator},
    _failed_tries_limit(failed_iter), _total_tries_limit(max_iter) {}

  double process_scan(const TransformedLaserScan &raw_scan,
                      const RobotPose &init_pose,
                      const GridMap &map,
                      RobotPoseDelta &pose_delta) override {
    do_for_each_observer([&init_pose, &raw_scan, &map](ObsPtr obs) {
      obs->on_matching_start(init_pose, raw_scan, map);
    });
    auto scan = scan_probability_estimator()->filter_scan(raw_scan.scan, map);

    unsigned failed_tries = 0, total_tries = 0;
    RobotPose best_pose = init_pose;

    auto best_pose_prob = scan_probability(scan, best_pose, map);

    do_for_each_observer([best_pose, scan, best_pose_prob](ObsPtr obs) {
      obs->on_scan_test(best_pose, scan, best_pose_prob);
      obs->on_pose_update(best_pose, scan, best_pose_prob);
    });

    while (failed_tries < _failed_tries_limit &&
           total_tries < _total_tries_limit) {
      total_tries++;

      RobotPose sampled_pose = best_pose;
      sample_pose(sampled_pose);
      double sampled_scan_prob = scan_probability(scan, sampled_pose, map);
      do_for_each_observer([sampled_pose, scan, sampled_scan_prob](ObsPtr obs) {
        obs->on_scan_test(sampled_pose, scan, sampled_scan_prob);
      });

      if (sampled_scan_prob <= best_pose_prob) {
        failed_tries++;
        continue;
      }

      best_pose_prob = sampled_scan_prob;
      best_pose = sampled_pose;
      failed_tries = on_estimate_update(failed_tries, _failed_tries_limit);

      do_for_each_observer([best_pose, scan, best_pose_prob](ObsPtr obs) {
        obs->on_pose_update(best_pose, scan, best_pose_prob);
      });
    }

    pose_delta = best_pose - init_pose;
    do_for_each_observer([pose_delta, best_pose_prob](ObsPtr obs) {
        obs->on_matching_end(pose_delta, best_pose_prob);
    });
    return best_pose_prob;
  }

protected:
  virtual void sample_pose(RobotPose &base_pose) = 0;
  virtual unsigned on_estimate_update(unsigned sample_num,
                                      unsigned sample_limit) = 0;

private:
  void do_for_each_observer(std::function<void(ObsPtr)> op) {
    for (auto &obs : GridScanMatcher::observers()) {
      if (auto obs_ptr = obs.lock()) {
        op(obs_ptr);
      }
    }
  }

private:
  unsigned _failed_tries_limit;
  unsigned _total_tries_limit;
};

#endif
