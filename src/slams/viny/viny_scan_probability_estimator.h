#ifndef SLAM_CTOR_SLAM_VINY_SCAN_COST_ESTIMATOR_H
#define SLAM_CTOR_SLAM_VINY_SCAN_COST_ESTIMATOR_H

#include "../../core/scan_matchers/weighted_mean_discrepancy_spe.h"

class VinyScanProbabilityEstimator : public WeightedMeanDiscrepancySPEstimator {
public:

  VinyScanProbabilityEstimator(OOPE oope)
    : WeightedMeanDiscrepancySPEstimator{oope} {}

  double scan_point_weight(const LaserScan2D::Points &pts,
                           LaserScan2D::Points::size_type i) const override {
    const auto &sp = pts[i];
    double angle = sp.angle();
    double weight = std::abs(std::sin(angle)) + std::abs(std::cos(angle));
    if (0.9 < std::abs(std::cos(angle))) {
      weight = 3;
    } else if (0.8 < std::abs(std::cos(angle))) {
      weight = 2;
    }
    return weight * std::sqrt(sp.range());
  }

};

#endif
