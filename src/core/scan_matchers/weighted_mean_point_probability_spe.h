#ifndef SLAM_CTOR_CORE_WEIGHTED_MEAN_DISCREPANCY_SP_ESTIMATOR
#define SLAM_CTOR_CORE_WEIGHTED_MEAN_DISCREPANCY_SP_ESTIMATOR

#include <cmath>
#include "grid_scan_matcher.h"
#include "../maps/grid_rasterization.h"
#include "../features/angle_histogram.h"

//============================================================================//
//                      Scan Point Weighting                                  //

class ScanPointWeighting {
public:
  using PointId = LaserScan2D::Points::size_type;
  virtual void reset(const LaserScan2D &scan) {}
  virtual double weight(const LaserScan2D::Points &, PointId) const = 0;
  virtual ~ScanPointWeighting() {}
};

class EvenSPW : public ScanPointWeighting {
public:
  void reset(const LaserScan2D &scan) override {
    _common_weight = 1.0 / scan.points().size();
  }

  double weight(const LaserScan2D::Points &, PointId) const override {
    return _common_weight;
  }
private:
  double _common_weight;
};

class AngleHistogramReciprocalSPW : public ScanPointWeighting {
public:
  void reset(const LaserScan2D &scan) override { _hist.reset(scan); }

  double weight(const LaserScan2D::Points &pts, PointId id) const override {
    auto v = _hist.value(pts, id);
    assert(v && "[BUG] AHR-SPW. Unknown point.");
    return 1.0 / v;
  }
private:
  AngleHistogram _hist;
};

class VinySlamSPW : public ScanPointWeighting {
public:
  double weight(const LaserScan2D::Points &pts, PointId id) const override {
    const auto &sp = pts[id];
    auto angle = sp.angle();
    auto weight = std::abs(std::sin(angle)) + std::abs(std::cos(angle));
    if (0.9 < std::abs(std::cos(angle))) {
      weight = 3;
    } else if (0.8 < std::abs(std::cos(angle))) {
      weight = 2;
    }
    return weight * std::sqrt(sp.range());
  }
};

//============================================================================//
//                       Weighted Mean Discrepancy SPE                        //

class WeightedMeanPointProbabilitySPE : public ScanProbabilityEstimator {
protected:
  using SPW = std::shared_ptr<ScanPointWeighting>;
public:
  WeightedMeanPointProbabilitySPE(OOPE oope, SPW spw)
    : ScanProbabilityEstimator{oope}, _spw{spw} {}

  LaserScan2D filter_scan(const LaserScan2D &raw_scan, const RobotPose &pose,
                          const GridMap &map) override {
    LaserScan2D scan;
    scan.trig_provider = raw_scan.trig_provider;
    scan.trig_provider->set_base_angle(pose.theta);

    auto &scan_pts = scan.points();
    for (const auto &sp : raw_scan.points()) {
      auto world_point = sp.move_origin(pose.x, pose.y, scan.trig_provider);
      auto area_id = map.world_to_cell(world_point);
      if (should_skip_point(sp, map, area_id)) { continue; }

      scan_pts.push_back(sp);
    }

    _spw->reset(scan);
    return scan;
  }

  double estimate_scan_probability(const LaserScan2D &scan,
                                   const RobotPose &pose,
                                   const GridMap &map,
                                   const SPEParams &params) const override {
    auto total_weight = double{0};
    auto total_probability = double{0};

    auto observation = expected_scan_point_observation();
    scan.trig_provider->set_base_angle(pose.theta);
    const auto &points = scan.points();
    for (LaserScan2D::Points::size_type i = 0; i < points.size(); ++i) {
      auto &sp = points[i];
      // FIXME: assumption - sensor pose is in robot's (0,0), dir - 0

      // prepare obstacle-based AreaOccupancyObservation
      observation.obstacle = params.scan_is_prerotated ?
        sp.move_origin(pose.x, pose.y) :
        sp.move_origin(pose.x, pose.y, scan.trig_provider);
      // area around the obstacle taken into account
      // Q: move obst_area to AOO?
      auto obs_area = params.sp_analysis_area.move_center(observation.obstacle);

      // estimate AOO probability
      auto aoo_prob = occupancy_observation_probability(observation,
                                                        obs_area, map);

      auto sp_weight = _spw->weight(points, i);
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

private:
  SPW _spw;
};

// The Feb 17 version

/* class WeightedMeanPointProbabilitySPE : public ScanProbabilityEstimator { */
/* public: */
/*   using SPW = std::shared_ptr<ScanPointWeighting>; */
/*    WeightedMeanPointProbabilitySPE(OOPE oope, SPW spw) */
/*      : ScanProbabilityEstimator{oope} {} */
/*   virtual double estimate_scan_probability( */
/*     const LaserScan2D &scan, */
/*     const RobotPose &pose, */
/*     const GridMap &map, */
/*     const SPEParams &params */
/*     ) const override { */
/*     auto OCCUPIED_OBSERVATION = AreaOccupancyObservation{ */
/*       true, {1.0, 1.0}, {0, 0}, 1.0}; */
/*     double cost = 0; */
/*     for (const auto &sp : scan.points()) { */
/*       if (!sp.is_occupied()) { */
/*         continue; */
/*       } */
/*       // move to world frame assume sensor coords (0,0) */
/*       double x_world = pose.x + sp.range() * std::cos(sp.angle()+pose.theta); */
/*       double y_world = pose.y + sp.range() * std::sin(sp.angle()+pose.theta); */
/*       //double cost_weight = 1.0; */
/*       //double cost_weight = std::abs(std::cos(sp.angle)) + 0.5;// * sp.range; */
/*       double cost_weight = std::abs(std::sin(sp.angle())) + */
/*         std::abs(std::cos(sp.angle())); */
/*       if (0.9 < std::abs(std::cos(sp.angle()))) { */
/*         cost_weight = 3; */
/*       } else if (0.8 < std::abs(std::cos(sp.angle()))) { */
/*         cost_weight = 2; */
/*       } */
/*       cost_weight *= std::sqrt(sp.range()); */
/*       DiscretePoint2D cell_coord = map.world_to_cell(x_world, y_world); */
/*       if (!map.has_cell(cell_coord)) { */
/*         cost += 1.0 * cost_weight; */
/*         continue; */
/*       } */

/*       cost += map[cell_coord].discrepancy(OCCUPIED_OBSERVATION) * cost_weight; */
/*     } */
/*     return cost; */
/*   } */

/* }; */


#endif
