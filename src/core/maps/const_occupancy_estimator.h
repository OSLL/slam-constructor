#ifndef SLAM_CTOR_CORE_CONST_OCCUPANCY_ESTIMATOR_H
#define SLAM_CTOR_CORE_CONST_OCCUPANCY_ESTIMATOR_H

#include "cell_occupancy_estimator.h"

class ConstOccupancyEstimator : public CellOccupancyEstimator {
public:
  ConstOccupancyEstimator(const Occupancy& base_occ,
                          const Occupancy& base_empty):
    CellOccupancyEstimator(base_occ, base_empty) {}
  virtual Occupancy estimate_occupancy(const Segment2D &beam,
                                       const Rectangle &cell_bnds,
                                       bool is_occ) override {
    return is_occ ? base_occupied() : base_empty();
  }

};

#endif
