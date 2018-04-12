#ifndef SLAM_CTOR_CORE_GRID_MAP_SCAN_ADDERS_H
#define SLAM_CTOR_CORE_GRID_MAP_SCAN_ADDERS_H

#include <cmath>
#include <limits>
#include <memory>

#include "grid_map.h"
#include "cell_occupancy_estimator.h"
#include "../states/sensor_data.h"
#include "../states/world.h"

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
    scan.trig_provider->set_base_angle(pose.theta);
    size_t last_pt_i = scan.points().size() - scan_margin - 1;
    for (size_t pt_i = scan_margin; pt_i <= last_pt_i; ++pt_i) {
      const auto &sp = scan.points()[pt_i];
      // move to world frame assume sensor is in robots' (0,0)
      const auto &wp = sp.move_origin(rp, scan.trig_provider);
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

public:
  std::shared_ptr<CellOccupancyEstimator> _occ_est;
};

class WallDistanceBlurringScanAdder : public GridMapScanAdder {
private:
  class WallDistanceBlurringScanAdderBuilder {
  #define ADD_SETTER(type, prop_name)                            \
    private:                                                     \
      type _##prop_name;                                         \
  public:                                                        \
    auto& set_##prop_name(decltype(_##prop_name) prop_name) {    \
      _##prop_name = prop_name;                                  \
      return *this;                                              \
    }                                                            \
    const auto& prop_name() const { return _##prop_name; }

    ADD_SETTER(double, blur_distance);
    ADD_SETTER(std::shared_ptr<CellOccupancyEstimator>, occupancy_estimator);
    ADD_SETTER(double, max_usable_range);
  #undef ADD_SETTER

  public:
    WallDistanceBlurringScanAdderBuilder()
      : _blur_distance{0}
      , _max_usable_range{std::numeric_limits<double>::infinity()} {}

    auto build() const {
      return std::make_shared<WallDistanceBlurringScanAdder>(*this);
    }
  };
public:
  static WallDistanceBlurringScanAdderBuilder builder() {
    return WallDistanceBlurringScanAdderBuilder{};
  }

  using ScanAdderProperties = WallDistanceBlurringScanAdderBuilder;
  WallDistanceBlurringScanAdder(const ScanAdderProperties &props)
    : GridMapScanAdder{props.occupancy_estimator()}
    , _props{props}
    , _max_usable_range_sq{std::pow(_props.max_usable_range(), 2)}{}
protected:

  // TODO: limit beam randering by distance
  //       either from a robot or a from obstace
  // TODO: consider renaming blur to distortion
  void handle_scan_point(GridMap &map, bool is_occ, double scan_quality,
                         const Segment2D &beam) const override {
    if (_max_usable_range_sq < beam.length_sq()) {
      return;
    }
    // TODO: simplify, consider performance if _blur_dist is not set
    //       (a static factory method)
    auto robot_pt = map.world_to_cell(beam.beg());
    auto obst_pt = map.world_to_cell(beam.end());
    auto obst_dist_sq = robot_pt.dist_sq(obst_pt);
    auto hole_dist_sq = std::pow(blur_cell_dist(map, beam, is_occ), 2);

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
  // return obstacle blur distance in cells
  double blur_cell_dist(const GridMap &map, const Segment2D &beam,
                        bool is_occ) const {
    if (!is_occ) { // no wall -> no blurring
      return 0;
    }

    auto blur_dist = _props.blur_distance() / map.scale();
    if (blur_dist < 0) {
      // dynamic wall blurring, Hole_Width is a scaling coefficient
      double dist_sq = beam.length_sq();
      blur_dist *= -dist_sq;
    }
    return blur_dist;
  }

private:
  ScanAdderProperties _props;
  double _max_usable_range_sq;
};

#endif
