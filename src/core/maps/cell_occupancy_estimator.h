#ifndef SLAM_CTOR_CORE_CELL_OCCUPANCY_ESTIMATOR_H
#define SLAM_CTOR_CORE_CELL_OCCUPANCY_ESTIMATOR_H

#include "../geometry_utils.h"
#include "../states/state_data.h" // the Occupancy class

class CellOccupancyEstimator {
public:
  CellOccupancyEstimator(const Occupancy& base_occ,
                         const Occupancy& base_empty):
    _base_occ(base_occ), _base_empty(base_empty) {}
  virtual ~CellOccupancyEstimator() = default;
  virtual Occupancy estimate_occupancy(const Segment2D &beam,
                                       const Rectangle &cell_bnds,
                                       bool is_occ) = 0;
protected:
  const Occupancy& base_occupied() const { return _base_occ; }
  const Occupancy& base_empty() const { return _base_empty; }
private:
  Occupancy _base_occ, _base_empty;
};

#endif
