#ifndef SLAM_CTOR_SLAM_VINY_SCAN_MATCHER_H_INCLUDED
#define SLAM_CTOR_SLAM_VINY_SCAN_MATCHER_H_INCLUDED

#include <random>

#include "../../core/scan_matchers/monte_carlo_scan_matcher.h"

struct VinySMParams {
  double sig_xy;
  double sig_th;
  unsigned bad_lmt;
  unsigned tot_lmt;
  unsigned seed;
  VinySMParams(double sig_xy, double sig_th, unsigned bad_lmt, unsigned tot_lmt,
               unsigned seed)
    : sig_xy(sig_xy), sig_th(sig_th), bad_lmt(bad_lmt), tot_lmt(tot_lmt)
    , seed(seed) {}
};

class VinyScanMatcher : public MonteCarloScanMatcher {
public:
  VinyScanMatcher(SPE prob_estimator, const VinySMParams& params)
    : MonteCarloScanMatcher(prob_estimator, params.bad_lmt, params.tot_lmt)
    , _sigma_coord(params.sig_xy), _sigma_angle(params.sig_th)
    , _curr_sigma_coord(_sigma_coord), _curr_sigma_angle(_sigma_angle)
    , _pr_generator(params.seed) {}

  void reset_state() override {
    _curr_sigma_coord = _sigma_coord;
    _curr_sigma_angle = _sigma_angle;
  }

protected:
  void sample_pose(RobotPose &base_pose) override {
    std::normal_distribution<> d_coord(0.0, _curr_sigma_coord);
    std::normal_distribution<> d_angle(0.0, _curr_sigma_angle);

    base_pose.x += d_coord(_pr_generator);
    base_pose.y += d_coord(_pr_generator);
    base_pose.theta += d_angle(_pr_generator);
  }

  unsigned on_estimate_update(unsigned sample_num,
                              unsigned sample_limit) override {
    if (sample_num <= sample_limit / 3) {
      return sample_num;
    }

    _curr_sigma_coord *= 0.5;
    _curr_sigma_angle *= 0.5;
    return 0;
  }

private:
  double _sigma_coord, _sigma_angle;
  double _curr_sigma_coord, _curr_sigma_angle;
  std::mt19937 _pr_generator;
};

#endif
