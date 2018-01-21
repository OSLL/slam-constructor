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

// The Feb 17 version

/* class MonteCarloScanMatcher : public GridScanMatcher { */
/* public: */
/*   MonteCarloScanMatcher(std::shared_ptr<ScanProbabilityEstimator> estimator, */
/*                          unsigned seed, */
/*                          double translation_dispersion, */
/*                          double rotation_dispersion, */
/*                          unsigned failed_attempts_per_dispersion, */
/*                          unsigned total_attempts) */
/*     : GridScanMatcher(estimator) */
/*     , _failed_tries_limit(failed_attempts_per_dispersion) */
/*     , _total_tries_limit(total_attempts) */
/*     , _sigma_coord(translation_dispersion), _sigma_angle(rotation_dispersion) */
/*     , _curr_sigma_coord(_sigma_coord), _curr_sigma_angle(_sigma_angle) */
/*     , _pr_generator(seed) {} */

/*   virtual double process_scan( */
/*                           const TransformedLaserScan &scan, */
/*     const RobotPose &init_pose, */
/*                       const GridMap &map, */
/*                       RobotPoseDelta &pose_delta) override { */

/*     unsigned failed_tries = 0, total_tries = 0; */
/*     auto sce = GridScanMatcher::scan_probability_estimator(); */
/*     double min_scan_cost = std::numeric_limits<double>::max(); */
/*     RobotPose optimal_pose = init_pose; */

/*     min_scan_cost = sce->estimate_scan_probability(scan.scan, optimal_pose, map); */


/*     while (failed_tries < _failed_tries_limit && */
/*            total_tries < _total_tries_limit) { */
/*       total_tries++; */

/*       RobotPose sampled_pose = optimal_pose; */
/*       sample_pose(sampled_pose); */
/*       double sampled_scan_cost = sce->estimate_scan_probability(scan.scan, sampled_pose, map); */

/*       if (min_scan_cost <= sampled_scan_cost) { */
/*         failed_tries++; */
/*         continue; */
/*       } */

/*       min_scan_cost = sampled_scan_cost; */
/*       optimal_pose = sampled_pose; */
/*       failed_tries = on_estimate_update(failed_tries, _failed_tries_limit); */

/*     } */

/*     pose_delta = optimal_pose - init_pose; */
/*     return min_scan_cost; */
/*   } */

/*   virtual void reset_state() override { */
/*     _curr_sigma_coord = _sigma_coord; */
/*     _curr_sigma_angle = _sigma_angle; */
/*   } */

/* protected: */
/*   void sample_pose(RobotPose &base_pose) { */
/*     std::normal_distribution<> d_coord(0.0, _curr_sigma_coord); */
/*     std::normal_distribution<> d_angle(0.0, _curr_sigma_angle); */

/*     base_pose.x += d_coord(_pr_generator); */
/*     base_pose.y += d_coord(_pr_generator); */
/*     base_pose.theta += d_angle(_pr_generator); */
/*   } */

/*   unsigned on_estimate_update(unsigned sample_num, */
/*                               unsigned sample_limit) { */
/*     if (sample_num <= sample_limit / 3) { */
/*       return sample_num; */
/*     } */

/*     _curr_sigma_coord *= 0.5; */
/*     _curr_sigma_angle *= 0.5; */
/*     return 0; */
/*   } */

/* private: */
/*   unsigned _failed_tries_limit; */
/*   unsigned _total_tries_limit; */
/*   double _sigma_coord, _sigma_angle; */
/*   double _curr_sigma_coord, _curr_sigma_angle; */
/*   std::mt19937 _pr_generator; */
/* }; */

#endif
