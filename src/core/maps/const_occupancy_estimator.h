#ifndef __CONST_OCCUPANCY_ESTIMATOR_H
#define __CONST_OCCUPANCY_ESTIMATOR_H

#include "cell_occupancy_estimator.h"

class ConstOccupancyEstimator : public CellOccupancyEstimator {
public:
  ConstOccupancyEstimator(double occ, double empty) :
    CellOccupancyEstimator(occ, empty) {}
  virtual Occupancy estimate_occupancy(const Segment2D &beam,
                                       const Rectangle &cell_bnds,
                                       bool is_occ) override {
    return Occupancy{is_occ ? base_occ_prob() : base_empty_prob(),
              is_occ ? 0.04 : 0.003};
  }

};

#endif
