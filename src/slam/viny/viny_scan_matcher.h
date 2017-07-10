#ifndef SLAM_CTOR_SLAM_VINY_SCAN_MATCHER_H_INCLUDED
#define SLAM_CTOR_SLAM_VINY_SCAN_MATCHER_H_INCLUDED

#include <random>

#include "../../core/scan_matchers/monte_carlo_scan_matcher.h"

struct VinySMParams {
  const double Sig_XY;
  const double Sig_TH;
  const unsigned Bad_Lmt;
  const unsigned Tot_Lmt;
  const unsigned Seed;
  VinySMParams(double sig_xy, double sig_th, unsigned bad_lmt, unsigned tot_lmt,
               unsigned seed)
    : Sig_XY(sig_xy), Sig_TH(sig_th), Bad_Lmt(bad_lmt), Tot_Lmt(tot_lmt)
    , Seed(seed) {}
};

class VinyScanMatcher : public MonteCarloScanMatcher {
public:
  VinyScanMatcher(SPE prob_estimator, const VinySMParams& params)
    : MonteCarloScanMatcher(prob_estimator, params.Bad_Lmt, params.Tot_Lmt)
    , _sigma_coord(params.Sig_XY), _sigma_angle(params.Sig_TH)
    , _curr_sigma_coord(_sigma_coord), _curr_sigma_angle(_sigma_angle)
    , _pr_generator(params.Seed) {}

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
