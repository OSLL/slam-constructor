#ifndef __GRID_CELL_H_INCLUDED
#define __GRID_CELL_H_INCLUDED

#include <memory>

#include "cell_occupancy_estimator.h"

class GridCell {
public:
  GridCell(const Occupancy &occ, double qual = 1.0) :
    occupancy{occ}, quality{qual} {}
  GridCell(const GridCell& gc) = default;
  GridCell& operator=(const GridCell& gc) = default;
  GridCell(GridCell&& gc) = default;
  GridCell& operator=(GridCell&& gc) = default;
  virtual ~GridCell() = default;

  operator double() const { return occupancy.prob_occ; }
  explicit operator bool() const { return occupancy.prob_occ == 0; }

  virtual std::unique_ptr<GridCell> clone() const {
    return std::make_unique<GridCell>(*this);
  }

  virtual void operator+=(const GridCell &that) {
    *this = that;
  }

public:
  Occupancy occupancy;
  double quality;
};

#endif
