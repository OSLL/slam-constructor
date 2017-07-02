#ifndef SLAM_CTOR_CORE_GRID_MAP_SCAN_ADDERS_H
#define SLAM_CTOR_CORE_GRID_MAP_SCAN_ADDERS_H

#include <cmath>
#include <memory>

#include "grid_map.h"
#include "cell_occupancy_estimator.h"
#include "../sensor_data.h"
#include "../world.h"

class GridMapScanAdder {
public:
  GridMapScanAdder(std::shared_ptr<CellOccupancyEstimator> e)
    : _occ_est{e} {}

  GridMap& append_scan(GridMap &map, const RobotPose &pose,
                       const LaserScan2D &scan,
                       double scan_quality,
                       double scan_margin = 0.0) const {
    if (scan.points().empty()) { return map; }

    const auto rp = pose.point();
    scan.trig_cache->set_theta(pose.theta);
    size_t last_pt_i = scan.points().size() - scan_margin - 1;
    for (size_t pt_i = scan_margin; pt_i <= last_pt_i; ++pt_i) {
      const auto &sp = scan.points()[pt_i];
      // move to world frame assume sensor is in robots' (0,0)
      const auto &wp = sp.move_origin(rp, scan.trig_cache);
      handle_scan_point(map, sp.is_occupied(), scan_quality, {rp, wp});
    }

    return map;
  }

protected:
  using AOO = AreaOccupancyObservation;
  auto estimate_occupancy(const Segment2D &beam,
                          const Rectangle &area_bnds,
                          bool is_occupied) const {
    return _occ_est->estimate_occupancy(beam, area_bnds, is_occupied);
  }

  virtual void handle_scan_point(GridMap &map, bool is_occ, double scan_quality,
                                 const Segment2D &beam) const = 0;

private:
  std::shared_ptr<CellOccupancyEstimator> _occ_est;
};

class WallDistanceBlurringScanAdder : public GridMapScanAdder {
public:
  WallDistanceBlurringScanAdder(std::shared_ptr<CellOccupancyEstimator> e,
                                double blur_dist)
    : GridMapScanAdder{e}, _blur_dist(blur_dist) {}
protected:

  void handle_scan_point(GridMap &map, bool is_occ, double scan_quality,
                         const Segment2D &beam) const override {

    // TODO: simplify, consider performance if _blur_dist is not set

    auto robot_pt = map.world_to_cell(beam.beg());
    auto obst_pt = map.world_to_cell(beam.end());
    auto obst_dist_sq = robot_pt.dist_sq(obst_pt);
    // TODO: hope dist estimation -> move to method
    auto hole_dist = _blur_dist / map.scale();
    if (hole_dist < 0) {
      // dynamic wall blurring, Hole_Width is a scaling coefficient
      double dist_sq = beam.length_sq();
      hole_dist *= -dist_sq;
    }
    if (!is_occ) {
      // no wall -> no blurring
      hole_dist = 0;
    }

    auto hole_dist_sq = std::pow(hole_dist, 2);

    auto pts = map.world_to_cells(beam);
    auto pt_bounds = map.world_cell_bounds(pts.back());
    auto base_occup = estimate_occupancy(beam, pt_bounds, is_occ);
    auto occ_aoo = AOO{is_occ, base_occup, beam.end(), scan_quality};
    map.update(pts.back(), occ_aoo);
    pts.pop_back();

    auto empty_aoo = AOO{false, {0, 0}, beam.end(), scan_quality};
    for (const auto &pt : pts) {
      const auto dist_sq = pt.dist_sq(obst_pt);
      auto pt_bounds = map.world_cell_bounds(pt);
      empty_aoo.is_occupied = false;
      empty_aoo.occupancy = estimate_occupancy(beam, pt_bounds, false);

      if (dist_sq < hole_dist_sq && hole_dist_sq < obst_dist_sq) {
        // NB: empty cell occupancy quality is not changed
        empty_aoo.is_occupied = true;
        auto prob_scale = 1.0 - dist_sq / hole_dist_sq;
        empty_aoo.occupancy.prob_occ = base_occup.prob_occ * prob_scale;
      }
      map.update(pt, empty_aoo);
    }
  }

private:
  double _blur_dist;
};

#endif
