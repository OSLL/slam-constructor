#ifndef SLAM_CTOR_CORE_GRID_CELL_H_INCLUDED
#define SLAM_CTOR_CORE_GRID_CELL_H_INCLUDED

#include <memory>
#include "../sensor_data.h"

class GridCell {
public:
  GridCell(const Occupancy &occ) : _occupancy{occ} {}
  GridCell(const GridCell& gc) = default;
  GridCell& operator=(const GridCell& gc) = default;
  GridCell(GridCell&& gc) = default;
  GridCell& operator=(GridCell&& gc) = default;
  virtual ~GridCell() = default;

  operator double() const { return occupancy().prob_occ; }
  explicit operator bool() const { return double(*this) == 0; }
  virtual const Occupancy& occupancy() const { return _occupancy; }

  virtual std::unique_ptr<GridCell> clone() const {
    return std::make_unique<GridCell>(*this);
  }

  virtual void operator+=(const AreaOccupancyObservation &aoo) {
    _occupancy = aoo.occupancy;
  }

  virtual double discrepancy(const AreaOccupancyObservation &aoo) const {
    return std::abs(_occupancy - aoo.occupancy);
  }

protected:
  Occupancy _occupancy;
};

#endif
