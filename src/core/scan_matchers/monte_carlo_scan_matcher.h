#ifndef SLAM_CTOR_CORE_MONTE_CARLO_SCAN_MATCHER_H
#define SLAM_CTOR_CORE_MONTE_CARLO_SCAN_MATCHER_H

#include <functional>
#include <limits>
#include <memory>
#include <random>

#include "../random_utils.h"
#include "grid_scan_matcher.h"

class PoseEnumerator {
public:
  virtual bool has_next() const = 0;
  virtual RobotPose next(const RobotPose &prev_pose) = 0;
  virtual void reset() {};
  virtual void feedback(bool /* pose_is_acceptable */) = 0;
  virtual ~PoseEnumerator() {}
};

class GaussianPoseEnumerator : public PoseEnumerator {
public:
  // TODO: refactonig - param names
  GaussianPoseEnumerator(double sigma_xy, double sigma_angle,
                         unsigned seed,
                         unsigned failed_samples_nm, unsigned total_samples_nm)
    : _sigma_coord_init{sigma_xy}, _sigma_angle_init{sigma_angle}
    , _failed_tries_limit{failed_samples_nm}
    , _total_tries_limit{total_samples_nm}
    , _pr_generator{seed} {
    reset();
  }

  bool has_next() const override {
    return _failed_tries_curr < _failed_tries_limit &&
           _total_tries_curr < _total_tries_limit;
  }

  RobotPose next(const RobotPose &prev_pose) override {
    auto shift = RobotPoseDelta{_coord_rv.sample(_pr_generator),
                                _coord_rv.sample(_pr_generator),
                                _angle_rv.sample(_pr_generator)};
    return prev_pose + shift;
  }

  void reset() override {
    _failed_tries_curr = _total_tries_curr = 0;
    _sigma_coord_curr = _sigma_coord_init;
    _sigma_angle_curr = _sigma_angle_init;
    update_rv();
  }

  void feedback(bool pose_is_acceptable) override {
    ++_total_tries_curr;
    if (!pose_is_acceptable) {
      ++_failed_tries_curr;
    } else {
      if (_failed_tries_curr <= _failed_tries_limit / 3) {
        // we haven't failed enough times to shrink the lookup area
        return;
      }
      _sigma_coord_curr *= 0.5;
      _sigma_angle_curr *= 0.5;
      _failed_tries_curr = 0;
      update_rv();
    }
  }
private:
  void update_rv() {
    _coord_rv = GaussianRV1D{0, _sigma_coord_curr};
    _angle_rv = GaussianRV1D{0, _sigma_angle_curr};
  }

private:
  // sampling params
  double _sigma_coord_init, _sigma_angle_init;
  double _sigma_coord_curr, _sigma_angle_curr;
  // TODO: use RobotPoseDeltaRV<std::mt19937>
  GaussianRV1D _coord_rv{0, 0}, _angle_rv{0, 0};
  // number of poses

  // TODO: rename, connect with sigma_curr
  unsigned _failed_tries_limit, _total_tries_limit;
  unsigned _failed_tries_curr, _total_tries_curr;

  std::mt19937 _pr_generator;
};

// TODO: merge the logic with hill climbing scan matcher
//       create free functions that create scan matchers
// TODO: move publish transform to observer
class MonteCarloScanMatcher : public GridScanMatcher {
public:
  MonteCarloScanMatcher(SPE estimator,
                        std::shared_ptr<PoseEnumerator> pose_enumerator)
    : GridScanMatcher{estimator}, _pose_enumerator{pose_enumerator} {}

  // add pose_enumerator setter

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

      auto pose_is_acceptable = sampled_scan_prob <= best_pose_prob;
      _pose_enumerator->feedback(pose_is_acceptable);
      if (pose_is_acceptable) {
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
