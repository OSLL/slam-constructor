#ifndef __GMAPPING_WORLD
#define __GMAPPING_WORLD

#include <random>
#include <cmath>

#include "../core/particle_filter.h"
#include "../core/laser_scan_grid_world.h"
#include "../core/maps/grid_cell_factory.h"
#include "../core/maps/grid_cell_strategy.h"

#include "scan_matcher.h"

class GmappingBaseCell : public GridCell {
public:
  GmappingBaseCell(): _hits(0), _tries(0), _phys_x(0), _phys_y(0) {}

  double value() const override {
    return _tries ? 1.0*_hits / _tries : -1;
  }

  void set_value(const Occupancy &occ, double) override {
    if (0.5 < occ.prob_occ) {
      _phys_x += occ.x;
      _phys_y += occ.y;
      ++_hits;
    }
    ++_tries;
  }

  double obst_x() const override { return _hits ? _phys_x / _hits : 0; }
  double obst_y() const override { return _hits ? _phys_y / _hits : 0; }

private:
  int _hits, _tries;
  double _phys_x, _phys_y;
};

class GmappingWorld : public Particle, public LaserScanGridWorld {
public:
  using MapType = GridMap;
  using Point2D = DiscretePoint2D;
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
