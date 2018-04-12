#ifndef SLAM_CTOR_SLAM_GMAPPING_GRID_CELL_H
#define SLAM_CTOR_SLAM_GMAPPING_GRID_CELL_H

#include <memory>
#include <cmath>
#include "../../core/geometry_utils.h"
#include "../../core/maps/grid_cell.h"

class GmappingBaseCell : public GridCell {
private:
  // TODO: move to param
  constexpr static double DIST_VARIANCE = 0.05;
public:
  GmappingBaseCell(): GridCell{Occupancy{-1, 1}}, _hits(0), _tries(0) {}

  std::unique_ptr<GridCell> clone() const override {
    return std::make_unique<GmappingBaseCell>(*this);
  }

  void operator+=(const AreaOccupancyObservation &aoo) override {
    if (!aoo.occupancy.is_valid()) { return; }

    ++_tries;
    bool aoo_is_free = aoo.occupancy <= 0.5;
    double aoo_p = aoo_is_free ? 0.0 : aoo.occupancy.prob_occ;
    _occupancy.prob_occ = ((*this) * (_tries - 1) + aoo_p) / _tries;
    if (aoo_is_free) return;

    ++_hits;
    obst.x = (obst.x * (_hits - 1) + aoo.obstacle.x) / _hits;
    obst.y = (obst.y * (_hits - 1) + aoo.obstacle.y) / _hits;
  }

  double discrepancy(const AreaOccupancyObservation &aoo) const override {
    auto similarity = std::exp(-obst.dist_sq(aoo.obstacle) / DIST_VARIANCE);
    return 1.0 - similarity;
  }

private:
  int _hits, _tries;
  Point2D obst;
};

#endif
