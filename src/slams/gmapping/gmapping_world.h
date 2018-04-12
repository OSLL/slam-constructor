#ifndef SLAM_CTOR_GMAPPING_WORLD_H
#define SLAM_CTOR_GMAPPING_WORLD_H

#include <memory>
#include <random>
#include <cmath>

#include "../../core/particle_filter.h"
#include "../../core/states/single_state_hypothesis_laser_scan_grid_world.h"
#include "../../core/maps/grid_cell.h"
#include "../../core/maps/lazy_tiled_grid_map.h"
#include "../../core/scan_matchers/hill_climbing_scan_matcher.h"

#include "gmapping_grid_cell.h"

struct GMappingParams {
private:
  using RandomEngine = std::mt19937;
  using GRV1D = GaussianRV1D<RandomEngine>;
  using URV1D = UniformRV1D<RandomEngine>;
public:
  RobotPoseDeltaRV<RandomEngine> pose_guess_rv, next_sm_delta_rv;

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

class GmappingWorld
  : public Particle
  , public SingleStateHypothesisLaserScanGridWorld<UnboundedLazyTiledGridMap> {
public:
  using RandomEngine = std::mt19937;
  using GRV1D = GaussianRV1D<RandomEngine>;
  using MapType = UnboundedLazyTiledGridMap;
public:

  GmappingWorld(const SingleStateHypothesisLSGWProperties &shw_params,
                const GMappingParams &gparams)
    // FIXME: [Performance] The adder does extra computations
    //        related to blurring that are not actually used
    : SingleStateHypothesisLaserScanGridWorld{shw_params}
    , _raw_odom_pose{0, 0, 0}
    , _rnd_engine(std::random_device{}())
    , _pose_guess_rv{gparams.pose_guess_rv}
    , _next_sm_delta_rv{gparams.next_sm_delta_rv} {
    reset_scan_matching_delta();
  }

  void update_robot_pose(const RobotPoseDelta& delta) override {
    auto d_th = (pose() - _raw_odom_pose).theta;
    auto s = std::sin(d_th), c = std::cos(d_th);
    // rotate delta's translation by d_th
    auto corrected_delta = RobotPoseDelta{
      c*delta.x - s*delta.y,
      s*delta.x + c*delta.y,
      delta.theta
      //atan2(std::sin(delta.theta), std::cos(delta.theta)
    };
    _raw_odom_pose += delta;

    _delta_since_last_sm += corrected_delta.abs();
    LaserScanGridWorld<MapType>::update_robot_pose(corrected_delta);
  }

  void handle_observation(TransformedLaserScan &scan) override {
    if (_delta_since_last_sm.sq_dist() < _next_sm_delta.sq_dist() &&
        std::fabs(_delta_since_last_sm.theta) < _next_sm_delta.theta) {
      return;
    }

    // TODO: consider super::handle_observation

    if (!_scan_is_first) {
      // add "noise" to guess extra cost function peak
      auto noise = _pose_guess_rv.sample(_rnd_engine);
      LaserScanGridWorld<MapType>::update_robot_pose(noise);
    }

    RobotPoseDelta pose_delta;
    double scan_prob = scan_matcher()->process_scan(scan, pose(),
                                                    map(), pose_delta);
    LaserScanGridWorld<MapType>::update_robot_pose(pose_delta);

    // TODO: scan_prob threshold to params
    if (0.0 < scan_prob || _scan_is_first) {
      // map update accordig to original gmapping code (ref?)
      scan_adder()->append_scan(map(), pose(), scan.scan, scan.quality, 0);
      _scan_is_first = false;
    }

    Particle::set_weight(scan_prob * Particle::weight());
    reset_scan_matching_delta();
  }

  void mark_master() {
    _is_master = true;
    // master is corrected on each step without noise
    _pose_guess_rv = RobotPoseDeltaRV<RandomEngine>{
      GRV1D{0, 0}, GRV1D{0, 0}, GRV1D{0, 0}};
    _next_sm_delta_rv = RobotPoseDeltaRV<RandomEngine>{
      GRV1D{0, 0}, GRV1D{0, 0}, GRV1D{0, 0}};
  }
  bool is_master() { return _is_master; }
  void sample() override { _is_master = false; }

private:

  void reset_scan_matching_delta() {
    _delta_since_last_sm.reset();
    _next_sm_delta = _next_sm_delta_rv.sample(_rnd_engine);
  }

private:
  RobotPose _raw_odom_pose;
  bool _is_master = false;
  bool _scan_is_first = true;
  RandomEngine _rnd_engine;
  RobotPoseDeltaRV<RandomEngine> _pose_guess_rv, _next_sm_delta_rv;
  RobotPoseDelta _delta_since_last_sm, _next_sm_delta;
};

#endif
