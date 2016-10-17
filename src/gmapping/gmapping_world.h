#ifndef __GMAPPING_WORLD
#define __GMAPPING_WORLD

#include <random>
#include <cmath>

#include "../core/particle_filter.h"
#include "../core/laser_scan_grid_world.h"
#include "../core/maps/grid_cell_factory.h"
#include "../core/maps/grid_cell_strategy.h"
#include "../core/gradient_walker_scan_matcher.h"

class GmappingCellValue : public GridCellValue {
public:
  GmappingCellValue() : GridCellValue(0, 0) {}

  virtual void reset() {
    obst.x = obst.y = 0;
    GridCellValue::reset();
  }

  Point2D obst;
};

// TODO: move grid cell models to a separate file
#include "gmapping_cost_estimator.h"

class GmappingBaseCell : public GridCell {
public:
  GmappingBaseCell(): _hits(0), _tries(0), _obst_x(0), _obst_y(0) {}

  const GridCellValue& value() const override {
    _out_value.occupancy.prob_occ = _tries ? 1.0*_hits / _tries : -1;
    // TODO: _hits == 0 -> return cell middle?
    _out_value.obst.x = _hits ? _obst_x / _hits : 0;
    _out_value.obst.y = _hits ? _obst_y / _hits : 0;
    return _out_value;
  }

  void set_value (const GridCellValue &new_value, double quality) override {
    ++_tries;
    if (new_value.occupancy <= 0.5) {
      return;
    }

    ++_hits;
    const GmappingCellValue &new_obs =
      dynamic_cast<const GmappingCellValue&>(new_value);
    _obst_x += new_obs.obst.x;
    _obst_y += new_obs.obst.y;
  }

private:
  mutable GmappingCellValue _out_value;
  int _hits, _tries;
  double _obst_x, _obst_y;
};

class GmappingWorld : public Particle, public LaserScanGridWorld {
public:
  using MapType = GridMap;
public:

  GmappingWorld(std::shared_ptr<GridCellStrategy> gcs) :
    LaserScanGridWorld(gcs),
    _matcher(std::make_shared<GmappingCostEstimator>()) {}

  virtual void handle_observation(TransformedLaserScan &scan) override {
    static bool scan_is_first = true;
    RobotPoseDelta pose_delta;
    double scan_score = _matcher.process_scan(pose(), scan, map(), pose_delta);
    update_robot_pose(pose_delta);

    // TODO: scan_prob threshold to params
    if (0.0 < scan_score || scan_is_first) {
      // map update accordig to original gmapping code (ref?)
      scan_is_first = false;
      LaserScanGridWorld::handle_observation(scan);
    }

    Particle::set_weight(scan_score / scan.points.size() * Particle::weight());
  }

  virtual GridCellValue& setup_cell_value(
      GridCellValue &dst, const DPoint &pt, const Rectangle &pt_bounds,
      bool is_occ, const Point2D &lsr, const Point2D &obstacle) override {

    if (is_occ) {
      GmappingCellValue &gmg_dst = dynamic_cast<GmappingCellValue&>(dst);
      gmg_dst.obst = obstacle;
    }
    LaserScanGridWorld::setup_cell_value(dst, pt, pt_bounds,
                                         is_occ, lsr, obstacle);
    return dst;
  }

  virtual void sample() override {
    std::random_device rd;
    std::mt19937 engine(rd());
    std::normal_distribution<> d_coord(0.0, 0.1);
    std::normal_distribution<> d_angle(0.0, 0.03);

   RobotPoseDelta norm_delta(d_coord(engine), d_coord(engine), d_angle(engine));
   LaserScanGridWorld::update_robot_pose(norm_delta);
  }

private:
  GradientWalkerScanMatcher _matcher;
};


#endif
