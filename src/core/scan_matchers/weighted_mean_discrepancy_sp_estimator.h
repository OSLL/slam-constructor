#ifndef SLAM_CTOR_CORE_WEIGHTED_MEAN_DISCREPANCY_SP_ESTIMATOR
#define SLAM_CTOR_CORE_WEIGHTED_MEAN_DISCREPANCY_SP_ESTIMATOR

#include "grid_scan_matcher.h"

class WeightedMeanDiscrepancySPEstimator : public ScanProbabilityEstimator {
public:
  double estimate_scan_probability(const TransformedLaserScan &tr_scan,
                                   const RobotPose &pose,
                                   const GridMap &map) const override {
    const auto OCCUPIED_AREA_OBS = expected_scan_point_observation();

    auto total_weight = double{0};
    auto total_probability = double{0};
    for (const auto &scan_point : tr_scan.scan.points()) {
      // FIXME: assumption - sensor pose is in robot's (0,0), dir - 0
      auto world_point = scan_point.move_origin(pose.x, pose.y, pose.theta);
      auto area_id = map.world_to_cell(world_point);
      if (should_skip_point(scan_point, map, area_id)) { continue; }

      auto sp_probability = 1.0 - map[area_id].discrepancy(OCCUPIED_AREA_OBS);
      auto sp_weight = scan_point_weight(scan_point);
      total_probability += sp_probability * sp_weight;
      total_weight += sp_weight;
    }
    if (total_weight == 0) { return unknown_probability(); }
    return total_probability / total_weight;
  }

protected:
  virtual AreaOccupancyObservation expected_scan_point_observation() const {
    return {true, {1.0, 1.0}, {0, 0}, 1.0};
  }

  virtual bool should_skip_point(const ScanPoint2D &sp,
                                 const GridMap &map,
                                 const GridMap::Coord &area_id) const {
    return !sp.is_occupied() || !map.has_cell(area_id);
  }

  virtual double scan_point_weight(const ScanPoint2D &) const {
    return 1;
  }
};

#endif
