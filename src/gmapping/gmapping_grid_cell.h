#ifndef __GMAPPING_GRID_CELL_H_INCLUDED
#define __GMAPPING_GRID_CELL_H_INCLUDED

#include <memory>
#include "../core/geometry_utils.h"
#include "../core/maps/grid_cell.h"

class GmappingBaseCell : public GridCell {
public:
  GmappingBaseCell(): GridCell{Occupancy{-1, 1}}, _hits(0), _tries(0) {}

  virtual std::unique_ptr<GridCell> clone() const override {
    return std::make_unique<GmappingBaseCell>(*this);
  }

  virtual void operator+=(const AreaOccupancyObservation &aoo) override {
    if (!aoo.occupancy.is_valid()) { return; }

    ++_tries;
    bool aoo_is_free = aoo.occupancy <= 0.5;
    double aoo_p = aoo_is_free ? 0.0 : aoo.occupancy.prob_occ;
    occupancy.prob_occ = ((*this) * (_tries - 1) + aoo_p) / _tries;
    if (aoo_is_free) return;

    ++_hits;
    obst.x = (obst.x * (_hits - 1) + aoo.obstacle.x) / _hits;
    obst.y = (obst.y * (_hits - 1) + aoo.obstacle.y) / _hits;
  }

  virtual double discrepancy(const AreaOccupancyObservation &aoo) const {
    return obst.dist_sq(aoo.obstacle);
  }

private:
  int _hits, _tries;
  Point2D obst;
};

#endif
