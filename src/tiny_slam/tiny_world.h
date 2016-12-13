#ifndef __TINY_WORLD_H
#define __TINY_WORLD_H

#include <iostream>
#include <cmath>
#include <memory>

#include "../core/world.h"
#include "../core/sensor_data.h"
#include "../core/laser_scan_grid_world.h"
#include "../core/maps/grid_cell_strategy.h"
#include "../core/maps/cell_occupancy_estimator.h"
#include "../core/maps/plain_grid_map.h"
#include "../core/maps/grid_cell.h"

#include "tiny_scan_matcher.h"

struct TinyWorldParams {
  double localized_scan_quality, raw_scan_quality;
  const TinySMParams Tiny_SM_Params;
  const double Hole_Width;

  TinyWorldParams(const TinySMParams& tiny_sm_params, double hole_width) :
    Tiny_SM_Params(tiny_sm_params), Hole_Width(hole_width) {}
};

class TinyWorld : public LaserScanGridWorld<UnboundedPlainGridMap> {
public:
  using Point = DiscretePoint2D;
public:

  TinyWorld(std::shared_ptr<GridCellStrategy> gcs,
            const TinyWorldParams &params,
            const GridMapParams &map_params) :
    LaserScanGridWorld(gcs, map_params), _gcs(gcs), _params(params),
    _scan_matcher(new TinyScanMatcher(_gcs->cost_est(), params.Tiny_SM_Params)),
    _map_update_ctx(_gcs->occupancy_est()) {}

  std::shared_ptr<GridScanMatcher> scan_matcher() override {
    return _scan_matcher;
  }

protected:

  void handle_observation(TransformedLaserScan &scan) override {
    _scan_matcher->reset_state();

    RobotPoseDelta pose_delta;
    _scan_matcher->process_scan(pose(), scan, map(), pose_delta);
    update_robot_pose(pose_delta);

    scan.quality = pose_delta ? _params.localized_scan_quality :
                                _params.raw_scan_quality;
    LaserScanGridWorld::handle_observation(scan);
  }

  void handle_scan_point(bool is_occ, double scan_quality,
                         const Segment2D &beam) override {
    auto &map = this->map();
    _map_update_ctx.blur_is_enabled = is_occ;

    DPoint robot_pt = map.world_to_cell(beam.beg());
    DPoint obst_pt = map.world_to_cell(beam.end());

    _map_update_ctx.obst_dist_sq = robot_pt.dist_sq(obst_pt);
    double hole_dist = _params.Hole_Width / map.scale();
    _map_update_ctx.hole_dist_sq = std::pow(hole_dist, 2);
    _map_update_ctx.obst_pt = obst_pt;

    LaserScanGridWorld::handle_scan_point(is_occ, scan_quality, beam);
  }

  AreaOccupancyObservation sp2obs(const DPoint &pt, bool is_occ, double quality,
                                  const Segment2D &beam) const override {
    AreaOccupancyObservation dst;
    dst.quality = quality;

    auto pt_bounds = map().world_cell_bounds(pt);
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
  mutable MapUpdateCtx _map_update_ctx;
};

#endif
