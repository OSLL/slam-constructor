#ifndef SLAM_CTOR_TINYSLAM_TINY_SCAN_COST_ESTIMATOR_H
#define SLAM_CTOR_TINYSLAM_TINY_SCAN_COST_ESTIMATOR_H

#include "../core/sensor_data.h"
#include "../core/scan_matchers/grid_scan_matcher.h"

// TODO: make the implementation base
class TinyScanProbabilityEstimator : public ScanProbabilityEstimator {
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
      // move to world frame assume sensor coords (0,0)
      auto coord = map.world_to_cell_by_vec(pose.x, pose.y,
                                            sp.range(), sp.angle()+pose.theta);
      if (!map.has_cell(coord)) {
        continue;
      }
      cost += 1.0 - map[coord].discrepancy(OCCUPIED_OBSERVATION);
    }
    // TODO: normalize [0, 1]
    return cost;
  }

};

#endif
