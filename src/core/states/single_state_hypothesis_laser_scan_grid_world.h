#ifndef SLAM_CTOR_CORE_SINGLE_STATE_HYPOTHESIS_LASER_SCAN_GRID_WORLD_H
#define SLAM_CTOR_CORE_SINGLE_STATE_HYPOTHESIS_LASER_SCAN_GRID_WORLD_H

#include <memory>
#include <utility>

#include "../maps/grid_map.h"
#include "../maps/grid_map_scan_adders.h"

#include "laser_scan_grid_world.h"

struct SingleStateHypothesisLSGWProperties {
  double localized_scan_quality = 1.0;
  double raw_scan_quality = 1.0;
  std::size_t scan_margin = 0;

  std::shared_ptr<GridCell> cell_prototype;
  std::shared_ptr<GridScanMatcher> gsm;
  std::shared_ptr<GridMapScanAdder> gmsa;
  GridMapParams map_props;
};

template <typename MapT>
class SingleStateHypothesisLaserScanGridWorld
  : public LaserScanGridWorld<MapT> {
public:
  using MapType = typename LaserScanGridWorld<MapT>::MapType;
  using Properties = SingleStateHypothesisLSGWProperties;
public:
  SingleStateHypothesisLaserScanGridWorld(const Properties &props)
    : _props{props}
    , _map{_props.cell_prototype->clone(), _props.map_props}
  , _map_update_ctx{_props.gmsa->_occ_est} {}

  // scan matcher access
  auto scan_matcher() { return _props.gsm; }

  void add_sm_observer(std::shared_ptr<GridScanMatcherObserver> obs) {
    auto sm = scan_matcher();
    if (sm) { sm->subscribe(obs); }
  }

  void remove_sm_observer(std::shared_ptr<GridScanMatcherObserver> obs) {
    auto sm = scan_matcher();
    if (sm) { sm->unsubscribe(obs); }
  }

  // scan adder access
  auto scan_adder() { return _props.gmsa; }

  // state access
  const MapType& map() const override { return _map; }
  using LaserScanGridWorld<MapT>::map; // enable non-const map access

  // TODO: return scan prob
  virtual void handle_observation(TransformedLaserScan &tr_scan) {
    auto sm = scan_matcher();
    sm->reset_state();

    auto pose_delta = RobotPoseDelta{};
    sm->process_scan(tr_scan, this->pose(), this->map(), pose_delta);
    this->update_robot_pose(pose_delta);

    tr_scan.quality = pose_delta ? _props.localized_scan_quality
                                 : _props.raw_scan_quality;

    this->do_mapping(tr_scan);
    //scan_adder()->append_scan(_map, this->pose(), tr_scan.scan,
    //                          tr_scan.quality, _props.scan_margin);
  }

  // The Feb 17 Version

  void do_mapping(TransformedLaserScan &scan) {
    const RobotPose& pose = World<TransformedLaserScan, MapType>::pose();

    scan.scan.trig_provider->set_base_angle(pose.theta);
    auto _scan_margin = 0;
    size_t last_pt_i = scan.scan.points().size() - _scan_margin - 1;
    for (size_t pt_i = _scan_margin; pt_i <= last_pt_i; ++pt_i) {
      auto &sp = scan.scan.points()[pt_i];
      // move to world frame assume sensor is in robots' (0,0)
      double c = scan.scan.trig_provider->cos(sp.angle());
      double s = scan.scan.trig_provider->sin(sp.angle());

      double x_world = pose.x + sp.range() * c;
      double y_world = pose.y + sp.range() * s;

      handle_scan_point(sp.is_occupied(), scan.quality,
                        {{pose.x, pose.y}, {x_world, y_world}});
    }
  }

  void handle_scan_point(bool is_occ, double scan_quality,
                         const Segment2D &beam) {
    auto HOLE_WIDTH = 0.3;
    auto &map = this->map();
    _map_update_ctx.blur_is_enabled = is_occ;

    auto robot_pt = map.world_to_cell(beam.beg());
    auto obst_pt = map.world_to_cell(beam.end());

    _map_update_ctx.obst_dist_sq = robot_pt.dist_sq(obst_pt);
    double hole_dist = HOLE_WIDTH / map.scale();
    if (hole_dist < 0) {
      // dynamic wall blurring, Hole_Width is a scaling coefficient
      double dist_sq = beam.length_sq();
      hole_dist *= -dist_sq;
    }
    _map_update_ctx.hole_dist_sq = std::pow(hole_dist, 2);
    _map_update_ctx.obst_pt = obst_pt;

    auto pts = map.world_to_cells(beam);
    map.update(pts.back(), sp2obs(pts.back(), is_occ, scan_quality, beam));
    pts.pop_back();

    for (const auto &pt : pts) {
      map.update(pt, sp2obs(pt, false, scan_quality, beam));
    }
  }

  AreaOccupancyObservation sp2obs(const DiscretePoint2D &pt, bool is_occ,
                                  double quality,
                                  const Segment2D &beam) const  {
    AreaOccupancyObservation dst;
    dst.is_occupied = is_occ;
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
      dst.is_occupied = true;
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

protected:
  Properties _props;
  MapType _map;
    mutable MapUpdateCtx _map_update_ctx;
};

#endif
