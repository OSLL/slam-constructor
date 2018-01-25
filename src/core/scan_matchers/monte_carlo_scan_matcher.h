#ifndef SLAM_CTOR_CORE_MONTE_CARLO_SCAN_MATCHER_H
#define SLAM_CTOR_CORE_MONTE_CARLO_SCAN_MATCHER_H

#include <random>
#include <memory>

#include "../random_utils.h"
#include "pose_enumeration_scan_matcher.h"

class GaussianPoseEnumerator : public PoseEnumerator {
protected:
  using Engine = std::mt19937;
public:
  GaussianPoseEnumerator(unsigned seed,
                         double translation_dispersion,
                         double rotation_dispersion,
                         unsigned max_dispersion_failed_attempts,
                         unsigned max_poses_nm)
    : _max_failed_attempts_per_shift{max_dispersion_failed_attempts}
    , _max_poses_nm{max_poses_nm}
    , _base_translation_dispersion{translation_dispersion}
    , _base_rotation_dispersion{rotation_dispersion}
    , _pose_shift_rv{GaussianRV1D<Engine>{0, 0},
                     GaussianRV1D<Engine>{0, 0},
                     GaussianRV1D<Engine>{0, 0}}
    , _pr_generator{seed} {
    reset();
  }

  bool has_next() const override {
    return _failed_attempts_per_shift < _max_failed_attempts_per_shift &&
           _poses_nm < _max_poses_nm;
  }

  RobotPose next(const RobotPose &prev_pose) override {
    return prev_pose + _pose_shift_rv.sample(_pr_generator);
  }

  void reset() override {
    _poses_nm = 0;
    reset_shift(_base_translation_dispersion, _base_rotation_dispersion);
  }

  void feedback(bool pose_is_acceptable) override {
    ++_poses_nm;
    if (!pose_is_acceptable) {
      ++_failed_attempts_per_shift;
    } else {
      if (_failed_attempts_per_shift <= _max_failed_attempts_per_shift / 3) {
        // we haven't failed enough times to shrink the lookup area
        return;
      }
      reset_shift(0.5);
    }
  }

private:
  void reset_shift(double new_translation_dispersion,
                   double new_rotation_dispersion) {
    _failed_attempts_per_shift = 0;
    _translation_dispersion = new_translation_dispersion;
    _rotation_dispersion = new_rotation_dispersion;
    _pose_shift_rv = {GaussianRV1D<Engine>{0, _translation_dispersion},
                      GaussianRV1D<Engine>{0, _translation_dispersion},
                      GaussianRV1D<Engine>{0, _rotation_dispersion}};
  }

  void reset_shift(double dispersion_scale_factor) {
    reset_shift(_translation_dispersion * 0.5, _rotation_dispersion * 0.5);
  }

private:
  // number of poses
  unsigned _max_failed_attempts_per_shift, _max_poses_nm;
  unsigned _failed_attempts_per_shift, _poses_nm;
  // sampling params
  double _base_translation_dispersion, _base_rotation_dispersion;
  double _translation_dispersion, _rotation_dispersion;

  RobotPoseDeltaRV<Engine> _pose_shift_rv;
  Engine _pr_generator;
};

class MonteCarloScanMatcher : public PoseEnumerationScanMatcher {
public:
  // FIXME: update enumerator on set_lookup_ranges update
  MonteCarloScanMatcher(std::shared_ptr<ScanProbabilityEstimator> estimator,
                        unsigned seed,
                        double translation_dispersion,
                        double rotation_dispersion,
                        unsigned failed_attempts_per_dispersion,
                        unsigned total_attempts)
    : PoseEnumerationScanMatcher{
        estimator,
        std::make_shared<GaussianPoseEnumerator>(
          seed, translation_dispersion, rotation_dispersion,
          failed_attempts_per_dispersion, total_attempts
        )
      } {}
};

#endif
