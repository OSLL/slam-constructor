#ifndef SLAM_CTOR_CORE_WEIGHTED_MEAN_DISCREPANCY_SP_ESTIMATOR
#define SLAM_CTOR_CORE_WEIGHTED_MEAN_DISCREPANCY_SP_ESTIMATOR

#include <cmath>
#include "grid_scan_matcher.h"
#include "../maps/grid_rasterization.h"

class WeightedMeanDiscrepancySPEstimator : public ScanProbabilityEstimator {
public:
  WeightedMeanDiscrepancySPEstimator(OOPE oope)
    : ScanProbabilityEstimator{oope} {}

  LaserScan2D filter_scan(const LaserScan2D &raw_scan, const RobotPose &pose,
                          const GridMap &map) override {
    LaserScan2D scan;
    scan.trig_cache = raw_scan.trig_cache;
    scan.trig_cache->set_theta(pose.theta);

    auto &scan_pts = scan.points();
    for (const auto &sp : raw_scan.points()) {
      auto world_point = sp.move_origin(pose.x, pose.y, scan.trig_cache);
      auto area_id = map.world_to_cell(world_point);
      if (should_skip_point(sp, map, area_id)) { continue; }

      scan_pts.push_back(sp);
    }
    return scan;
  }

  double estimate_scan_probability(const LaserScan2D &scan,
                                   const RobotPose &pose,
                                   const GridMap &map,
                                   const SPEParams &params) const override {
    auto total_weight = double{0};
    auto total_probability = double{0};

    auto observation = expected_scan_point_observation();
    scan.trig_cache->set_theta(pose.theta);
    for (const auto &sp : scan.points()) {
      // FIXME: assumption - sensor pose is in robot's (0,0), dir - 0

      // prepare obstacle-based AreaOccupancyObservation
      observation.obstacle = params.scan_is_prerotated ?
        sp.move_origin(pose.x, pose.y) :
        sp.move_origin(pose.x, pose.y, scan.trig_cache);
      // area around the obstacle taken into account
      // Q: move obst_area to AOO?
      auto obs_area = params.sp_analysis_area.move_center(observation.obstacle);

      // estimate AOO probability
      auto aoo_prob = occupancy_observation_probability(observation,
                                                        obs_area, map);

      // combine AOO probability
      auto sp_weight = scan_point_weight(sp);
      total_probability += aoo_prob * sp_weight * sp.factor();
      total_weight += sp_weight;
    }
    if (total_weight == 0) {
      // TODO: replace with writing to a proper logger
      std::clog << "WARNING: unknown probability" << std::endl;
      return unknown_probability();
    }
    return total_probability / total_weight;
  }

protected:
  virtual AreaOccupancyObservation expected_scan_point_observation() const {
    // TODO: use a strategy to convert obstacle->occupancy
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
