#ifndef SLAM_CTOR_VINYSLAM_SCAN_COST_ESTIMATOR_H
#define SLAM_CTOR_VINYSLAM_SCAN_COST_ESTIMATOR_H

#include "../core/sensor_data.h"
#include "../core/scan_matchers/grid_scan_matcher.h"

class VinyScanProbabilityEstimator : public ScanProbabilityEstimator {
public:
  double estimate_scan_probability(const TransformedLaserScan &tr_scan,
                                   const RobotPose &pose,
                                   const GridMap &map) const override {
    auto OCCUPIED_OBSERVATION = AreaOccupancyObservation{
      true, {1.0, 1.0}, {0, 0}, 1.0};
    double cost = 0;
    for (const auto &sp : tr_scan.scan.points()) {
      if (!sp.is_occupied()) {
        continue;
      }
      double sp_angle = sp.angle();
      // move to world frame assume sensor coords (0,0)
      double x_world = pose.x + sp.range() * std::cos(sp_angle+pose.theta);
      double y_world = pose.y + sp.range() * std::sin(sp_angle+pose.theta);
      //double cost_weight = 1.0;
      //double cost_weight = std::abs(std::cos(sp.angle)) + 0.5;// * sp.range;
      double cost_weight = std::abs(std::sin(sp_angle)) +
                           std::abs(std::cos(sp_angle));
      if (0.9 < std::abs(std::cos(sp_angle))) {
        cost_weight = 3;
      } else if (0.8 < std::abs(std::cos(sp_angle))) {
        cost_weight = 2;
      }
      cost_weight *= std::sqrt(sp.range());
      DiscretePoint2D cell_coord = map.world_to_cell(x_world, y_world);
      if (!map.has_cell(cell_coord)) {
        continue;
      }

      cost += (1.0 - map[cell_coord].discrepancy(OCCUPIED_OBSERVATION))
                 * cost_weight;
    }
    // TODO: normalization
    return cost;
  }

};

#endif
