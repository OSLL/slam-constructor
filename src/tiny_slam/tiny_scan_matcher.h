#ifndef __TINY_SCAN_MATCHER_H
#define __TINY_SCAN_MATCHER_H

#include <random>

#include "../core/monte_carlo_scan_matcher.h"
#include "tiny_scan_cost_estimator.h"

struct TinySMParams {
  const double Sig_XY;
  const double Sig_TH;
  const unsigned Bad_Lmt;
  const unsigned Tot_Lmt;
  TinySMParams(double sig_xy, double sig_th,
               unsigned bad_lmt, unsigned tot_lmt)
    : Sig_XY(sig_xy), Sig_TH(sig_th)
    , Bad_Lmt(bad_lmt), Tot_Lmt(tot_lmt) {}
};

class TinyScanMatcher : public MonteCarloScanMatcher {
private:
  using ScePtr = std::shared_ptr<ScanCostEstimator>;
public:
  TinyScanMatcher(ScePtr cost_estimator, const TinySMParams& params):
   MonteCarloScanMatcher(cost_estimator, params.Bad_Lmt, params.Tot_Lmt),
    _sigma_coord(params.Sig_XY), _sigma_angle(params.Sig_TH),
    _curr_sigma_coord(_sigma_coord), _curr_sigma_angle(_sigma_angle) {}

  virtual void reset_state() override {
    _curr_sigma_coord = _sigma_coord;
    _curr_sigma_angle = _sigma_angle;
  }

protected:
  virtual void sample_pose(RobotPose &base_pose) override {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> d_coord(0.0, _curr_sigma_coord);
    std::normal_distribution<> d_angle(0.0, _curr_sigma_angle);

    base_pose.x += d_coord(gen);
    base_pose.y += d_coord(gen);
    base_pose.theta += d_angle(gen);
  }

  virtual unsigned on_estimate_update(unsigned sample_num,
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
};

#endif
