#ifndef __TINY_GRID_CELL_H
#define __TINY_GRID_CELL_H

#include "../core/maps/grid_cell.h"

//------------------------------------------------------------------------------
// Base cell

class BaseTinyCell : public GridCell {
public:
  BaseTinyCell(): GridCell{Occupancy{0.5, 1}} {}

  virtual std::unique_ptr<GridCell> clone() const {
    return std::make_unique<BaseTinyCell>(*this);
  }

  virtual void operator+=(const GridCell &that) {
    const double q = that.quality;
    occupancy.prob_occ = (1.0 - q) * (*this) + q * that;
  }
};

//------------------------------------------------------------------------------
// Modified cell

class AvgTinyCell : public GridCell {
public:
  AvgTinyCell(): GridCell{Occupancy{-1, 1}}, _n(0) {}

  virtual std::unique_ptr<GridCell> clone() const {
    return std::make_unique<AvgTinyCell>(*this);
  }

  virtual void operator+=(const GridCell &that) {
    _n += 1;
    double that_p = 0.5 + (that - 0.5) * that.quality;
    occupancy.prob_occ = ((*this) * (_n - 1) + that_p) / _n;
  }

private:
  double _n;
};

#endif
