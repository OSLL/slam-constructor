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

class GmappingWorld : public Particle,
                      public LaserScanGridWorld<UnboundedLazyTiledGridMap> {
public:
  using MapType = UnboundedLazyTiledGridMap;
public:

  GmappingWorld(std::shared_ptr<GridCellStrategy> gcs)
    : LaserScanGridWorld(gcs)
    , _matcher{std::make_shared<GmappingCostEstimator>()}
    , _rnd_engine(std::random_device{}()) {
    init_pose_delta();
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
    std::normal_distribution<> d_coord(0.0, 0.1);
    std::normal_distribution<> d_angle(0.0, 0.03);

    RobotPoseDelta norm_dlt(d_coord(_rnd_engine), d_coord(_rnd_engine),
                            d_angle(_rnd_engine));
    LaserScanGridWorld::update_robot_pose(norm_dlt);
  }

private:
  void init_pose_delta() {
    std::uniform_real_distribution<> d_coord(0.6, 0.8);
    std::uniform_real_distribution<> d_angle(0.3, 0.4);
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
};

#endif
