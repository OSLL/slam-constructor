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

  virtual void operator+=(const GridCell &that) override {
    ++_tries;
    bool that_is_free = that <= 0.5;
    double that_p = that_is_free ? 0 : that;
    occupancy.prob_occ = ((*this) * (_tries - 1) + that_p) / _tries;
    if (that_is_free) return;

    // use static cast for performance reasons
    auto gmg_that = static_cast<const GmappingBaseCell&>(that);
    ++_hits;
    obst.x = (obst.x * (_hits - 1) + gmg_that.obst.x) / _hits;
    obst.y = (obst.y * (_hits - 1) + gmg_that.obst.y) / _hits;
  }
public:
  Point2D obst;
private:
  int _hits, _tries;
};

#endif
