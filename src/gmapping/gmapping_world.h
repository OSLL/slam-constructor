#ifndef SLAM_CTOR_GMAPPING_WORLD_H_INCLUDED
#define SLAM_CTOR_GMAPPING_WORLD_H_INCLUDED

#include <memory>
#include <random>
#include <cmath>

#include "../core/particle_filter.h"
#include "../core/laser_scan_grid_world.h"
#include "../core/maps/grid_cell.h"
#include "../core/maps/grid_cell_strategy.h"
#include "../core/maps/lazy_tiled_grid_map.h"
#include "../core/scan_matchers/hill_climbing_scan_matcher.h"

#include "gmapping_grid_cell.h"
#include "gmapping_cost_estimator.h"

struct GMappingParams {
private:
  using GRV1D = GaussianRV1D;
  using URV1D = UniformRV1D;
public:
  RobotPoseDeltaRV<std::mt19937> pose_guess_rv, next_sm_delta_rv;

  GMappingParams(double mean_sample_xy, double sigma_sample_xy,
                 double mean_sample_th, double sigma_sample_th,
                 double min_sm_lim_xy, double max_sm_lim_xy,
                 double min_sm_lim_th, double max_sm_lim_th)
    : pose_guess_rv{GRV1D{mean_sample_xy, sigma_sample_xy},
                    GRV1D{mean_sample_xy, sigma_sample_xy},
                    GRV1D{mean_sample_th, sigma_sample_th}}
    , next_sm_delta_rv{URV1D{min_sm_lim_xy, max_sm_lim_xy},
                       URV1D{min_sm_lim_xy, max_sm_lim_xy},
                       URV1D{min_sm_lim_th, max_sm_lim_th}} {}
};

class GmappingWorld : public Particle,
                      public LaserScanGridWorld<UnboundedLazyTiledGridMap> {
public:
  using MapType = UnboundedLazyTiledGridMap;
public:

  GmappingWorld(std::shared_ptr<GridCellStrategy> gcs,
                const GridMapParams& params,
                const GMappingParams& gparams)
    : LaserScanGridWorld(gcs, params)
    , _matcher{std::make_shared<GmappingCostEstimator>()}
    , _rnd_engine(std::random_device{}())
    , _pose_guess_rv{gparams.pose_guess_rv}
    , _next_sm_delta_rv{gparams.next_sm_delta_rv} {
    reset_scan_matching_delta();
  }

  void update_robot_pose(const RobotPoseDelta& delta) override {
    _delta_since_last_sm += delta.abs();
    LaserScanGridWorld<MapType>::update_robot_pose(delta);
  }

  void handle_observation(TransformedLaserScan &scan) override {
    if (_delta_since_last_sm.sq_dist() < _next_sm_delta.sq_dist() &&
        std::fabs(_delta_since_last_sm.theta) < _next_sm_delta.theta) {
      return;
    }

    if (!_scan_is_first) {
      // add "noise" to guess extra cost function peak
      update_robot_pose(_pose_guess_rv.sample(_rnd_engine));
    }

    RobotPoseDelta pose_delta;
    double scan_score = _matcher.process_scan(pose(), scan, map(), pose_delta);
    update_robot_pose(pose_delta);

    double scan_prob = scan_score / scan.points.size();
    // TODO: scan_prob threshold to params
    if (0.0 < scan_prob || _scan_is_first) {
      // map update accordig to original gmapping code (ref?)
      LaserScanGridWorld::handle_observation(scan);
      _scan_is_first = false;
    }

    Particle::set_weight(scan_prob * Particle::weight());
    reset_scan_matching_delta();
  }

  void mark_master() {
    _is_master = true;
    // master is corrected on each step without noise
    _pose_guess_rv = RobotPoseDeltaRV<std::mt19937>{
      GaussianRV1D{0, 0}, GaussianRV1D{0, 0}, GaussianRV1D{0, 0}};
    _next_sm_delta_rv = RobotPoseDeltaRV<std::mt19937>{
      GaussianRV1D{0, 0}, GaussianRV1D{0, 0}, GaussianRV1D{0, 0}};
  }
  bool is_master() { return _is_master; }
  void sample() override { _is_master = false; }

private:

  void reset_scan_matching_delta() {
    _delta_since_last_sm.reset();
    _next_sm_delta = _next_sm_delta_rv.sample(_rnd_engine);
  }

private:
  bool _is_master = false;
  bool _scan_is_first = true;
  HillClimbingScanMatcher _matcher;
  std::mt19937 _rnd_engine;
  RobotPoseDeltaRV<std::mt19937> _pose_guess_rv, _next_sm_delta_rv;
  RobotPoseDelta _delta_since_last_sm, _next_sm_delta;
};

#endif
