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
#include "../core/gradient_walker_scan_matcher.h"

#include "gmapping_grid_cell.h"
#include "gmapping_cost_estimator.h"

template<typename T>
class RandomVariable1D {
public:
  virtual double sample(T& rnd_engine) = 0;
  virtual ~RandomVariable1D() {}
};

class GaussianRV1D : public RandomVariable1D<std::mt19937> {
public:
  GaussianRV1D(double mean, double sigma)
    : _mean{mean}, _sigma{sigma}, _distr{_mean, _sigma} {}

  double sample(std::mt19937 &rnd_engine) override {
    return _distr(rnd_engine);
  }
private:
  double _mean;
  double _sigma;
  std::normal_distribution<> _distr;
};

class UniformRV1D : public RandomVariable1D<std::mt19937> {
public:
  UniformRV1D(double from, double to)
    : _from{from}, _to{to}, _distr{_from, _to} {}

  double sample(std::mt19937 &rnd_engine) {
    return _distr(rnd_engine);
  }
private:
  double _from;
  double _to;
  std::uniform_real_distribution<> _distr;
};

struct NDP {
  double mean;
  double sigma;
  NDP(const double x0, const double sigma) : mean(x0), sigma(sigma) {}
};

struct GMappingParams {
  const NDP sample_xy;
  const NDP sample_th;
  const NDP init_pd_xy;
  const NDP init_pd_th;

  GMappingParams(const NDP& sample_xy, const NDP& sample_th,
                 const NDP& init_pd_xy, const NDP& init_pd_th)
    : sample_xy(sample_xy), sample_th(sample_th)
    , init_pd_xy(init_pd_xy), init_pd_th(init_pd_th) {}
  GMappingParams(const double mean_sample_xy, const double sigma_sample_xy,
                 const double mean_sample_th, const double sigma_sample_th,
                 const double mean_init_pd_xy, const double sigma_init_pd_xy,
                 const double mean_init_pd_th, const double sigma_init_pd_th)
    : sample_xy(mean_sample_xy, sigma_sample_xy)
    , sample_th(mean_sample_th, sigma_sample_th)
    , init_pd_xy(mean_init_pd_xy, sigma_init_pd_xy)
    , init_pd_th(mean_init_pd_th, sigma_init_pd_th) {}
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
    , _is_master{false}
    , _matcher{std::make_shared<GmappingCostEstimator>()}
    , _rnd_engine(std::random_device{}())
    , _sample_xy{gparams.sample_xy}, _sample_th{gparams.sample_th}
    , _init_pd_xy{gparams.init_pd_xy}, _init_pd_th{gparams.init_pd_th} {
    reset_pose_delta();
  }

  void update_robot_pose(const RobotPoseDelta& delta) override {
    _pose_delta += delta.abs();
    LaserScanGridWorld<MapType>::update_robot_pose(delta);
  }

  void handle_observation(TransformedLaserScan &scan) override {
    if (_pose_delta.sq_dist() < _dist_sq_lim &&
        std::fabs(_pose_delta.theta) < _ang_lim) {
      return;
    }

    if (!(_scan_is_first || _is_master)) {
      // add "noise" to a non-master particle in order
      // to guess extra cost function peak.
      sample_robot_pose();
    }

    RobotPoseDelta pose_delta;
    double scan_score = _matcher.process_scan(pose(), scan, map(), pose_delta);
    update_robot_pose(pose_delta);
    reset_pose_delta();

    double scan_prob = scan_score / scan.points.size();
    // TODO: scan_prob threshold to params
    if (0.0 < scan_prob || _scan_is_first) {
      // map update accordig to original gmapping code (ref?)
      LaserScanGridWorld::handle_observation(scan);
      _scan_is_first = false;
    }

    Particle::set_weight(scan_prob * Particle::weight());
  }

  void mark_master() { _is_master = true; }
  bool is_master() { return _is_master; }
  void sample() override { _is_master = false; }

private:

  void sample_robot_pose() {
    std::normal_distribution<> d_coord(_sample_xy.mean,
                                       _sample_xy.sigma);
    std::normal_distribution<> d_angle(_sample_th.mean,
                                       _sample_th.sigma);
    auto norm_dlt = RobotPoseDelta{d_coord(_rnd_engine), d_coord(_rnd_engine),
                                   d_angle(_rnd_engine)};
    LaserScanGridWorld::update_robot_pose(norm_dlt);
  }

  void reset_pose_delta() {
    std::uniform_real_distribution<> d_coord(_init_pd_xy.mean,
                                             _init_pd_xy.sigma);
    std::uniform_real_distribution<> d_angle(_init_pd_th.mean,
                                             _init_pd_th.sigma);
    _dist_sq_lim = d_coord(_rnd_engine);
    _ang_lim = d_angle(_rnd_engine);
    _pose_delta.reset();
  }

private:
  bool _is_master;
  GradientWalkerScanMatcher _matcher;
  RobotPoseDelta _pose_delta;
  double _dist_sq_lim, _ang_lim;
  bool _scan_is_first = true;
  std::mt19937 _rnd_engine;
  NDP _sample_xy, _sample_th, _init_pd_xy, _init_pd_th;
};

#endif
