#ifndef SLAM_CTOR_SLAM_TINY_GRID_CELL_H
#define SLAM_CTOR_SLAM_TINY_GRID_CELL_H

#include "../../core/maps/grid_cell.h"

//------------------------------------------------------------------------------
// Base cell

class BaseTinyCell : public GridCell {
public:
  BaseTinyCell(): GridCell{Occupancy{0.5, 1}} {}

  virtual std::unique_ptr<GridCell> clone() const {
    return std::make_unique<BaseTinyCell>(*this);
  }

  virtual void operator+=(const AreaOccupancyObservation &aoo) {
    if (!aoo.occupancy.is_valid()) { return; }

    const double q = aoo.quality;
    _occupancy.prob_occ = (1.0 - q) * (*this) + q * aoo.occupancy;
  }
};

//------------------------------------------------------------------------------
// Modified cell

class AvgTinyCell : public GridCell {
public:
  AvgTinyCell(): GridCell{Occupancy{0.5, 1}}, _n(0) {}

  virtual std::unique_ptr<GridCell> clone() const {
    return std::make_unique<AvgTinyCell>(*this);
  }

  virtual void operator+=(const AreaOccupancyObservation &aoo) {
    if (!aoo.occupancy.is_valid()) { return; }

    _n += 1;
    double that_p = 0.5 + (aoo.occupancy - 0.5) * aoo.quality;
    _occupancy.prob_occ = ((*this) * (_n - 1) + that_p) / _n;
  }

private:
  double _n;
};

#endif
