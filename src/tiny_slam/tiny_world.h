#ifndef __TINY_WORLD_H
#define __TINY_WORLD_H

#include <iostream>
#include <cmath>
#include <memory>

#include "../core/state_data.h"
#include "../core/sensor_data.h"
#include "../core/laser_scan_grid_world.h"
#include "../core/maps/grid_cell_strategy.h"
#include "../core/maps/cell_occupancy_estimator.h"
#include "../core/maps/plain_grid_map.h"
#include "../core/maps/grid_cell.h"

#include "tiny_scan_matcher.h"

struct TinyWorldParams {
  double localized_scan_quality, raw_scan_quality;
};

class TinyWorld : public LaserScanGridWorld<PlainGridMap> {
private: // internal params
  // Scan matcher
  const double SIG_XY = 0.2;
  const double SIG_TH = 0.1;
  const double BAD_LMT = 20;
  const double TOT_LMT = BAD_LMT * 5;

  const double HOLE_WIDTH = 1.5;
public:
  using Point = DiscretePoint2D;
public:

  TinyWorld(std::shared_ptr<GridCellStrategy> gcs,
            const TinyWorldParams &params) :
    LaserScanGridWorld(gcs), _gcs(gcs), _params(params),
    _scan_matcher(new TinyScanMatcher(_gcs->cost_est(),
                                      BAD_LMT, TOT_LMT,
                                      SIG_XY, SIG_TH)),
    _map_update_ctx(_gcs->occupancy_est()) {}

  virtual void handle_observation(TransformedLaserScan &scan) override {
    _scan_matcher->reset_state();

    RobotPoseDelta pose_delta;
    _scan_matcher->process_scan(pose(), scan, map(), pose_delta);
    update_robot_pose(pose_delta);

    scan.quality = pose_delta ? _params.localized_scan_quality :
                                _params.raw_scan_quality;
    LaserScanGridWorld::handle_observation(scan);
  }


  std::shared_ptr<GridScanMatcher> scan_matcher() {
    return _scan_matcher;
  }

  virtual void handle_scan_point(MapType &map, bool is_occ, double scan_quality,
    const Point2D &lsr, const Point2D &beam_end) override {

    _map_update_ctx.blur_is_enabled = is_occ;
    _map_update_ctx.beam = Beam{lsr.x, lsr.y, beam_end.x, beam_end.y};

    DPoint robot_pt = map.world_to_cell(lsr.x, lsr.y);
    DPoint obst_pt = map.world_to_cell(beam_end.x, beam_end.y);
    _map_update_ctx.obst_dist_sq = robot_pt.dist_sq(obst_pt);
    _map_update_ctx.hole_dist_sq = std::pow(HOLE_WIDTH / map.cell_scale(), 2);
    _map_update_ctx.obst_pt = obst_pt;

    LaserScanGridWorld::handle_scan_point(map, is_occ, scan_quality,
                                          lsr, beam_end);
  }

  virtual GridCell& setup_cell_value(
    GridCell &dst, const DPoint &pt, const Rectangle &pt_bounds,
    bool is_occ, const Point2D &lsr, const Point2D &obstacle) override {

    const Beam &beam = _map_update_ctx.beam;
    auto occ_est = _map_update_ctx.occ_est;
    dst.occupancy = occ_est->estimate_occupancy(beam, pt_bounds, is_occ);
    if (is_occ) {
      _map_update_ctx.base_occupied_prob = dst.occupancy;
      return dst;
    }

    if (!_map_update_ctx.blur_is_enabled) { return dst; }

    // perform wall blur
    const double hole_dist_sq = _map_update_ctx.hole_dist_sq;
    const double obst_dist_sq = _map_update_ctx.obst_dist_sq;
    const double dist_sq = pt.dist_sq(_map_update_ctx.obst_pt);

    if (dist_sq < hole_dist_sq && hole_dist_sq < obst_dist_sq) {
      double prob_scale = 1.0 - dist_sq / hole_dist_sq;
      dst.occupancy.prob_occ = prob_scale * _map_update_ctx.base_occupied_prob;
    }
    return dst;
  }

 private: // types
  struct MapUpdateCtx {
    MapUpdateCtx(std::shared_ptr<CellOccupancyEstimator> e) : occ_est(e) {}

    Rectangle cell_bounds;
    Beam beam;
    DiscretePoint2D obst_pt;
    double base_occupied_prob;
    double obst_dist_sq, hole_dist_sq;

    bool blur_is_enabled;
    std::shared_ptr<CellOccupancyEstimator> occ_est;
  };

private:
  std::shared_ptr<GridCellStrategy> _gcs;
  const TinyWorldParams _params;
  std::shared_ptr<TinyScanMatcher> _scan_matcher;

  // a context set up for each map update with a scan point
  MapUpdateCtx _map_update_ctx;
};

#endif
