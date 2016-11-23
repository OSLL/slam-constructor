#ifndef __GMAPPING_WORLD_H_INCLUDED
#define __GMAPPING_WORLD_H_INCLUDED

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

struct NDP {
  const double x0;
  const double sigma;

  NDP(const double x0, const double sigma) : x0(x0), sigma(sigma) {}
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
  GMappingParams(const double x0_sample_xy, const double sigma_sample_xy,
                 const double x0_sample_th, const double sigma_sample_th,
                 const double x0_init_pd_xy, const double sigma_init_pd_xy,
                 const double x0_init_pd_th, const double sigma_init_pd_th)
    : sample_xy(x0_sample_xy, sigma_sample_xy)
    , sample_th(x0_sample_th, sigma_sample_th)
    , init_pd_xy(x0_init_pd_xy, sigma_init_pd_xy)
    , init_pd_th(x0_init_pd_th, sigma_init_pd_th) {}
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
    , _gprms(gparams){
    init_pose_delta();
  }

  GmappingWorld operator = (const GmappingWorld& gw) {
    _matcher = gw._matcher;
    _pose_delta = gw._pose_delta;
    dist_sq_lim = gw.dist_sq_lim;
    ang_lim = gw.ang_lim;
    _scan_is_first = gw._scan_is_first;
    _rnd_engine = gw._rnd_engine;
    return *this;
  }
  virtual void update_robot_pose(const RobotPoseDelta& delta) {
    _pose_delta += delta;
    LaserScanGridWorld<MapType>::update_robot_pose(delta);
  }

  virtual void handle_observation(TransformedLaserScan &scan) override {

    if (_pose_delta.sq_dist() < dist_sq_lim &&
        std::fabs(_pose_delta.theta) < ang_lim) {
      return;
    }

    RobotPoseDelta pose_delta;
    double scan_score = _matcher.process_scan(pose(), scan, map(), pose_delta);
    update_robot_pose(pose_delta);
    init_pose_delta();

    double scan_prob = scan_score / scan.points.size();
    // TODO: scan_prob threshold to params
    if (0.0 < scan_prob || _scan_is_first) {
      // map update accordig to original gmapping code (ref?)
      LaserScanGridWorld::handle_observation(scan);
      _scan_is_first = false;
    }

    Particle::set_weight(scan_prob * Particle::weight());
  }

  virtual void sample() override {
    std::normal_distribution<> d_coord(_gprms.sample_xy.x0,
                                       _gprms.sample_xy.sigma);
    std::normal_distribution<> d_angle(_gprms.sample_th.x0,
                                       _gprms.sample_th.sigma);

    RobotPoseDelta norm_dlt(d_coord(_rnd_engine), d_coord(_rnd_engine),
                            d_angle(_rnd_engine));
    LaserScanGridWorld::update_robot_pose(norm_dlt);
  }

private:
  void init_pose_delta() {
    std::uniform_real_distribution<> d_coord(_gprms.init_pd_xy.x0,
                                             _gprms.init_pd_xy.sigma);
    std::uniform_real_distribution<> d_angle(_gprms.init_pd_th.x0,
                                             _gprms.init_pd_th.sigma);
    dist_sq_lim = d_coord(_rnd_engine);
    ang_lim = d_angle(_rnd_engine);
    _pose_delta.reset();
  }
private:
  GradientWalkerScanMatcher _matcher;
  RobotPoseDelta _pose_delta;
  double dist_sq_lim, ang_lim;
  bool _scan_is_first = true;
  std::mt19937 _rnd_engine;
  const GMappingParams _gprms;
};

#endif
