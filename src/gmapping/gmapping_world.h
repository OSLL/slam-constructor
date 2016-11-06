#ifndef __GMAPPING_WORLD
#define __GMAPPING_WORLD

#include <memory>
#include <random>
#include <cmath>

#include "../core/particle_filter.h"
#include "../core/laser_scan_grid_world.h"
#include "../core/maps/grid_cell_factory.h"
#include "../core/maps/grid_cell_strategy.h"
#include "../core/maps/lazy_layered_grid_map.h"
#include "../core/gradient_walker_scan_matcher.h"

class GmappingCellValue : public GridCellValue {
public:
  GmappingCellValue() : GridCellValue(0, 0) {}
  Point2D obst;
};

// TODO: move grid cell models to a separate file
#include "gmapping_cost_estimator.h"

class GmappingBaseCell : public GridCell {
public:
  GmappingBaseCell(): _hits(0), _tries(0), _obst_x(0), _obst_y(0) {
    update_value();
  }

  const GridCellValue& value() const override {
    return _out_value;
  }

  void set_value (const GridCellValue &new_value, double quality) override {
    ++_tries;
    if (new_value.occupancy <= 0.5) {
      update_value();
      return;
    }

    ++_hits;
    // use static cast for performance reasons
    const GmappingCellValue &new_obs =
      static_cast<const GmappingCellValue&>(new_value);
    _obst_x += new_obs.obst.x;
    _obst_y += new_obs.obst.y;
    update_value();
  }

  virtual std::shared_ptr<GridCell> clone() const {
    auto cloned = std::make_shared<GmappingBaseCell>();
    *cloned = *this;
    return cloned;
  }

private:
  void update_value() {
    _out_value.occupancy.prob_occ = _tries ? 1.0*_hits / _tries : -1;
    _out_value.obst.x = _hits ? _obst_x / _hits : 0;
    _out_value.obst.y = _hits ? _obst_y / _hits : 0;
  }
private:
  GmappingCellValue _out_value;
  int _hits, _tries;
  double _obst_x, _obst_y;
};

class GmappingWorld : public Particle,
                      public LaserScanGridWorld<LazyLayeredGridMap> {
public:
  using MapType = LazyLayeredGridMap;
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

  virtual GridCellValue& setup_cell_value(
      GridCellValue &dst, const DPoint &pt, const Rectangle &pt_bounds,
      bool is_occ, const Point2D &lsr, const Point2D &obstacle) override {

    if (is_occ) {
      // static cast is used for performance reasons
      GmappingCellValue &gmg_dst = static_cast<GmappingCellValue&>(dst);
      gmg_dst.obst = obstacle;
    }
    LaserScanGridWorld::setup_cell_value(dst, pt, pt_bounds,
                                         is_occ, lsr, obstacle);
    return dst;
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
