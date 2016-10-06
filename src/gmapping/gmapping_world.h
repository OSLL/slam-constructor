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

class GmappingBaseCellFactory : public GridCellFactory {
public:
  std::shared_ptr<GridCell> create_cell() override {
    return std::shared_ptr<GridCell>(new GmappingBaseCell());
  }
};

class GmappingWorld : public Particle, public LaserScanGridWorld {
public:
  using MapType = GridMap;
  using Point2D = DiscretePoint2D;
public:

  GmappingWorld(std::shared_ptr<GridCellStrategy> gcs) :
    LaserScanGridWorld(gcs) {}

  virtual void handle_observation(TransformedLaserScan &scan) override {
    // TODO: localization (scan matching)
    // -- ScanMatcher::optimize
    // -- ScanMatcher::likelihooddistanceAndScore (?)

    RobotPose pose_delta;
    double _scan_score = _matcher.processScan(pose(), scan, map(), pose_delta);
    update_robot_pose(scan.pose_delta);

    // add scan to the map
    Particle::set_weight(_scan_score / scan.points.size() * Particle::weight());
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
