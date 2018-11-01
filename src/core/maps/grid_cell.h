#ifndef SLAM_CTOR_CORE_GRID_CELL_H
#define SLAM_CTOR_CORE_GRID_CELL_H

#include <memory>
#include "../math_utils.h"
#include "../states/sensor_data.h"
#include "../serialization.h"

class GridCell {
public:
  GridCell(const Occupancy &occ) : _occupancy{occ}, _is_unknown{true} {}
  GridCell(const GridCell& gc) = default;
  GridCell& operator=(const GridCell& gc) = default;
  GridCell(GridCell&& gc) = default;
  GridCell& operator=(GridCell&& gc) = default;
  virtual ~GridCell() = default;

  operator double() const { return occupancy().prob_occ; }
  explicit operator bool() const { return are_equal(double(*this), 0.0); }
  virtual const Occupancy& occupancy() const { return _occupancy; }
  bool is_unknown() const { return _is_unknown; }

  virtual std::unique_ptr<GridCell> clone() const {
    return std::make_unique<GridCell>(*this);
  }

  virtual void operator+=(const AreaOccupancyObservation &aoo) {
    _occupancy = aoo.occupancy;
    _is_unknown = false;
  }

  // must be in interval [0, 1.0]
  virtual double discrepancy(const AreaOccupancyObservation &aoo) const {
    return std::abs(_occupancy - aoo.occupancy);
  }

  virtual std::vector<char> serialize() const {
      Serializer s; s << _occupancy.prob_occ << _occupancy.estimation_quality;
      return s.result();
  }

  virtual size_t deserialize(const std::vector<char>& data, size_t pos = 0) {
      Deserializer d(data, pos);
      d >> _occupancy.prob_occ >> _occupancy.estimation_quality;
      return d.pos();
  }

protected:
  Occupancy _occupancy;
  bool _is_unknown;
};

#endif
