#ifndef SLAM_CTOR_CORE_WEIGHTED_MEAN_DISCREPANCY_SP_ESTIMATOR
#define SLAM_CTOR_CORE_WEIGHTED_MEAN_DISCREPANCY_SP_ESTIMATOR

#include <cmath>
#include "grid_scan_matcher.h"

class WeightedMeanDiscrepancySPEstimator : public ScanProbabilityEstimator {
public:

  virtual LaserScan2D filter_scan(const LaserScan2D &raw_scan,
                                  const GridMap &map) {
    LaserScan2D scan;
    scan.trig_cache = raw_scan.trig_cache;

    auto &scan_pts = scan.points();
    for (const auto &scan_point : raw_scan.points()) {
      auto area_id = map.world_to_cell(scan_point.x(), scan_point.y());
      if (should_skip_point(scan_point, map, area_id)) { continue; }

      scan_pts.push_back(scan_point);
    }

    return scan;
  }

  double estimate_scan_probability(const LaserScan2D &scan,
                                   const RobotPose &pose,
                                   const GridMap &map,
                                   const SPEParams &params) const override {
    auto total_weight = double{0};
    auto total_probability = double{0};

    for (const auto &scan_point : scan.points()) {
      // FIXME: assumption - sensor pose is in robot's (0,0), dir - 0
      auto world_point = scan_point.move_origin(pose.x, pose.y, pose.theta);
      auto sp_prob = world_point_probability(world_point, pose, map, params);
      auto sp_weight = scan_point_weight(scan_point);
      total_probability += sp_prob * sp_weight;
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

  double world_point_probability(const Point2D &wp,
                                 const RobotPose &pose, const GridMap &map,
                                 const SPEParams &params) const {
    const auto OCCUPIED_AREA_OBS = expected_scan_point_observation();
    auto analysis_area = params.sp_analysis_area.move_center(wp);
    double highest_probability = 0;
    for (auto &area_id : map.coords_in_area(analysis_area)) {
      double area_prob = 1.0 - map[area_id].discrepancy(OCCUPIED_AREA_OBS);
      highest_probability = std::max(area_prob, highest_probability);
    }
    return highest_probability;
  }
};

// NB: experimental
/*
class FastMeanDiscrepancySPEstimator : public ScanProbabilityEstimator {
public:
  double estimate_scan_probability(const TransformedLaserScan &tr_scan,
                                   const RobotPose &pose,
                                   const GridMap &map,
                                   const SPEParams &params) const override {
    // TODO: filter non occupied points
    auto total_weight = double{0};
    auto total_probability = double{0};
    tr_scan.scan.trig_cache->set_theta(pose.theta);
    //int id = 0;
    for (const auto &scan_point : tr_scan.scan.points()) {
      //id++;
      //      if (id % 5) { continue; }
      // FIXME: assumption - sensor pose is in robot's (0,0), dir - 0
      //auto world_point = scan_point.move_origin(pose.x, pose.y);
      double c = tr_scan.scan.trig_cache->cos(scan_point.angle());
      double s = tr_scan.scan.trig_cache->sin(scan_point.angle());
      Point2D world_point = {pose.x + scan_point.range() * c,
                             pose.y + scan_point.range() * s};


      auto sp_prob = world_point_probability(world_point, pose, map, params);
      total_probability += sp_prob;
      total_weight += 1;
    }
    if (total_weight == 0) {
      // TODO: replace with writing to a proper logger
      std::clog << "WARNING: unknown probability" << std::endl;
      return unknown_probability();
    }
    return total_probability / total_weight;
  }

protected:

  double world_point_probability(const Point2D &wp,
                                 const RobotPose &pose, const GridMap &map,
                                 const SPEParams &params) const {
    auto analysis_area = params.sp_analysis_area.move_center(wp);
    double highest_probability = 0;
    for (auto &area_id : map.coords_in_area(analysis_area)) {
      highest_probability = std::max(double(map[area_id]), highest_probability);
    }
    return highest_probability;
  }
};
*/

#endif
