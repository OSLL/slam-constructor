#ifndef __GMAPPING_WORLD
#define __GMAPPING_WORLD

#include <random>
#include <cmath>

#include "../core/particle_filter.h"
#include "../core/laser_scan_grid_world.h"
#include "../core/maps/grid_cell_factory.h"
#include "../core/maps/grid_cell_strategy.h"

class GmappingCellValue : public GridCellValue {
public:
  GmappingCellValue() : GridCellValue(0, 0), obst_x(0), obst_y(0) {}

  virtual void reset() {
    obst_x = obst_y = 0;
    GridCellValue::reset();
  }

  double obst_x, obst_y;
};

// TODO: move grid cell models to a separate file
#include "scan_matcher.h"

class GmappingBaseCell : public GridCell {
public:
  GmappingBaseCell(): _hits(0), _tries(0), _obst_x(0), _obst_y(0) {}

  const GridCellValue& value() const override {
    _out_value.occupancy.prob_occ = _tries ? 1.0*_hits / _tries : -1;
    // TODO: _hits == 0 -> return cell middle?
    _out_value.obst_x = _hits ? _obst_x / _hits : 0;
    _out_value.obst_y = _hits ? _obst_y / _hits : 0;
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
    _obst_x += new_obs.obst_x;
    _obst_y += new_obs.obst_y;
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
    LaserScanGridWorld(gcs) {}

  virtual void handle_observation(TransformedLaserScan &scan) override {
    RobotPose pd;
    static bool scan_is_first = true;
    // TODO: fix SM iface
    double scan_score = _matcher.processScan(pose(), scan, map(), pd);
    RobotPoseDelta pose_delta(pd.x, pd.y, pd.theta);
    update_robot_pose(pose_delta);
    if (pose_delta || scan_is_first) {
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
      gmg_dst.obst_x = obstacle.x;
      gmg_dst.obst_y = obstacle.y;
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
  ScanMatcher _matcher;
};


#endif
