#ifndef _CELL_OCCUPANCY_ESTIMATOR_H
#define _CELL_OCCUPANCY_ESTIMATOR_H

#include "../geometry_utils.h"
#include "../state_data.h" // Occupancy class

class CellOccupancyEstimator {
public:
  CellOccupancyEstimator(double base_occ_prob, double base_empty_prob):
    _base_occ_prob(base_occ_prob), _base_empty_prob(base_empty_prob) {}
  virtual ~CellOccupancyEstimator() = default;
  virtual Occupancy estimate_occupancy(const Segment2D &beam,
                                       const Rectangle &cell_bnds,
                                       bool is_occ) = 0;
protected:
  double base_occ_prob() { return _base_occ_prob; }
  double base_empty_prob() { return _base_empty_prob; }
private:
  double _base_occ_prob, _base_empty_prob;
};

#endif
