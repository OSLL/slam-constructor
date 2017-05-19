#ifndef SLAM_CTOR_CORE_GRID_APPROXIMATOR_H_INCLUDED
#define SLAM_CTOR_CORE_GRID_APPROXIMATOR_H_INCLUDED

#include <limits>
#include <cassert>
#include <vector>
#include <memory>
#include <utility>

#include "regular_squares_grid.h"
#include "grid_map.h"
#include "grid_cell.h"

//------------------------------------------------------------------------------
// Approximated cells.
// * discrepancy is used to determine if the cell's content should be updated

class MaxSeenOccupancyGridCell : public GridCell {
public:
  MaxSeenOccupancyGridCell(double prob = 0)
    : GridCell{Occupancy{prob, 0}} {}

  std::unique_ptr<GridCell> clone() const override {
    return std::make_unique<MaxSeenOccupancyGridCell>(*this);
  }

  void operator+=(const AreaOccupancyObservation &aoo) override {
    _occupancy = aoo.occupancy;
  }

  double discrepancy(const AreaOccupancyObservation &aoo) const {
    return occupancy().prob_occ < aoo.occupancy.prob_occ;
  }
};

//------------------------------------------------------------------------------
// Fine <-> Coarse cell mapping

class ApproximationPolicy {
public:
  using RSGrid = RegularSquaresGrid;
  using Coords = std::vector<GridMap::Coord>;
protected:
  static constexpr double Dbl_Inf = std::numeric_limits<double>::infinity();
public:
  // returns coords in _dst_ that correspond to the point's coord in _src_
  virtual Coords project_point(const RSGrid &src, const RSGrid &dst,
                               const Point2D &point) const = 0;
  virtual std::shared_ptr<GridCell> approximated_area_proto() const = 0;
  virtual ~ApproximationPolicy() {}
};

class PlainMaxApproximationPolicy : public ApproximationPolicy {
public:
  PlainMaxApproximationPolicy(double dflt_prob)
    : _apprx_area{std::make_shared<MaxSeenOccupancyGridCell>(dflt_prob)} {}

  Coords project_point(const RSGrid &src, const RSGrid &dst,
                       const Point2D &point) const override {
    if (src.scale() <= dst.scale()) { // approximate
      return {dst.world_to_cell(point)};
    }

    // 'restore'; dst.scale() < src.scale()
    if (src.scale() == Dbl_Inf) {
      return dst.valid_coords();
    }
    return dst.coords_in_area(src.world_cell_bounds(src.world_to_cell(point)));
  }

  std::shared_ptr<GridCell> approximated_area_proto() const {
    return _apprx_area;
  }
private:
  std::shared_ptr<GridCell> _apprx_area;
};

//------------------------------------------------------------------------------
// Occupancy Grid Map Approximators

class OccupancyGridMapApproximator : public GridMapUpdateObserver {
public:
  using UpdatedPoints = std::vector<Point2D>;
public:
  OccupancyGridMapApproximator(std::shared_ptr<ApproximationPolicy> &policy)
    : _policy{policy} {}

  void on_grid_map_update(const GridMap& map,
                          const RegularSquaresGrid::Coord &area_id) override {};

  virtual unsigned max_approximation_level(const GridMap &) const = 0;
  virtual const GridMap& map(unsigned apprx_lvl, const GridMap&) const = 0;

  virtual ~OccupancyGridMapApproximator() {}

  static void watch_master_map(const GridMap& map,
      std::shared_ptr<OccupancyGridMapApproximator> approx) {
    map.observe_updates(approx);
    approx->set_master_map(map);
  }

protected:
  virtual void set_master_map(const GridMap& map) = 0;
  const ApproximationPolicy& policy() const {
    return *_policy;
  }
private:
  std::shared_ptr<ApproximationPolicy> _policy;
};

template <typename ApproxMapT, unsigned ScaleChangeRatio = 2>
class PowNCachedOGMA : public OccupancyGridMapApproximator {
private:
  static constexpr int Coarsest_Map_W = 1, Coarsest_Map_H = 1;
  static constexpr double Dbl_Inf = std::numeric_limits<double>::infinity();
public:
  PowNCachedOGMA(std::shared_ptr<ApproximationPolicy> policy)
    : OccupancyGridMapApproximator{policy} {}

  unsigned max_approximation_level(const GridMap &) const override {
    return _approximated_maps.size() - 1;
  }

  const GridMap& map(unsigned lvl, const GridMap &map) const override {
    assert(0 <= lvl && lvl <= max_approximation_level(map));
    return *_approximated_maps[lvl];
  }

  void on_grid_map_update(const GridMap& updated_map,
                          const RegularSquaresGrid::Coord &area_id) override {
    auto master_occ = updated_map[area_id].occupancy();
    auto virtual_pt = updated_map.cell_to_world(area_id);
    auto master_aoo = AreaOccupancyObservation{true, master_occ, virtual_pt, 1};
    // TODO: use unordered set
    std::vector<Point2D> unhandled_pts{virtual_pt};
    for (unsigned al = 0; al <= max_approximation_level(updated_map); ++al) {
      if (unhandled_pts.empty()) { break; }

      auto &coarse_map = modifiable_map(al, updated_map);
      decltype(unhandled_pts) next_lvl_pts;
      for (auto &pt : unhandled_pts) {
        auto coords = policy().project_point(updated_map, coarse_map, pt);
        for (auto &coarser_coord : coords) {
          if (!coarse_map[coarser_coord].discrepancy(master_aoo)) {
            continue;
          }
          coarse_map.update(coarser_coord, master_aoo);
          next_lvl_pts.push_back(coarse_map.cell_to_world(coarser_coord));
        } // for coarser_coord
      } // for pts
      unhandled_pts = std::move(next_lvl_pts);
    }
    ensure_approximation_cache_continuous(updated_map);
  };

protected:

  void set_master_map(const GridMap& map) override {
    assert(!_master_map_is_set && "Master map resent is not supported (yet).");
    _master_map_is_set = true;

    _approximated_maps.clear();

    // TODO: init approximations according to map content
    // TODO: rm map copy
    auto finest_mps = GridMapParams{map.width(), map.height(), map.scale()};
    _approximated_maps.push_back(make_approx_map(finest_mps));
    auto coarsest_mps = GridMapParams{Coarsest_Map_W, Coarsest_Map_H, Dbl_Inf};
    _approximated_maps.push_back(make_approx_map(coarsest_mps));
    ensure_approximation_cache_continuous(map);
  }

private:

  GridMap& modifiable_map(unsigned lvl, const GridMap &fine_map) {
    auto const_this = static_cast<const decltype(this)>(this);
    return const_cast<GridMap&>(const_this->map(lvl, fine_map));
  }

  std::unique_ptr<ApproxMapT> make_approx_map(const GridMapParams &gmp) const {
    auto area_proto = policy().approximated_area_proto();
    return std::make_unique<ApproxMapT>(area_proto, gmp);
  }

  void ensure_approximation_cache_continuous(const GridMap &fine_map) const {
    static const int Scale_Factor = ScaleChangeRatio;
    static const int PC_W_Target = Coarsest_Map_W * Scale_Factor,
                     PC_H_Target = Coarsest_Map_H * Scale_Factor;

    auto &pre_coarsest_map = map(max_approximation_level(fine_map) - 1,
                                 fine_map);
    int pc_w = pre_coarsest_map.width(), pc_h = pre_coarsest_map.height();
    double pc_scale = pre_coarsest_map.scale();

    if (pc_w <= PC_W_Target && pc_h <= PC_H_Target) { return; }

    pc_w = ge_pow<Scale_Factor>(pc_w), pc_h = ge_pow<Scale_Factor>(pc_h);
    // more cache levels have to be added
    while (PC_W_Target < pc_w || PC_H_Target < pc_h) {
      pc_w = std::max(PC_W_Target, int(std::ceil(pc_w / Scale_Factor)));
      pc_h = std::max(PC_H_Target, int(std::ceil(pc_h / Scale_Factor)));
      pc_scale *= Scale_Factor;

      _approximated_maps.insert((_approximated_maps.rbegin() + 1).base(),
                                make_approx_map({pc_w, pc_h, pc_scale}));
    }
  }

private:
  mutable std::vector<std::unique_ptr<GridMap>> _approximated_maps;
  std::shared_ptr<GridCell> _approximated_area_proto;
  bool _master_map_is_set = false;
};

#endif
