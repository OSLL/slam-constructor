#ifndef SLAM_CTOR_SLAM_VINY_TBM_PROB_CONVERSION_H
#define SLAM_CTOR_SLAM_VINY_TBM_PROB_CONVERSION_H

#include "../../core/maps/transferable_belief_model.h"
#include "../../core/states/state_data.h"

// TBM ---> Occupancy
Occupancy TBM_to_O(const TBM& tbm) {
  /* double qual = tbm.occupied() + tbm.empty(); */
  /* double p_occu = tbm.occupied() / qual; */
  /* return Occupancy { p_occu, qual }; */
  return Occupancy{ tbm.occupied() + 0.5 * tbm.unknown(), 1.0 };
}

// AreaOccupancyObservation ---> TBM
TBM AOO_to_TBM(const AreaOccupancyObservation& aoo) {
  if (!aoo.occupancy.is_valid())
    return TBM();
  
  // TODO: consider other conversion schemas
  double prob_occ = aoo.occupancy.prob_occ;
  double est_qual = aoo.occupancy.estimation_quality * aoo.quality;
  
  double occupied = prob_occ * est_qual;
  double empty = (1 - prob_occ) * est_qual;
  return TBM{1.0 - occupied - empty, empty, occupied, 0.0};
  
  /*
  if (aoo.is_occupied) {
    _occupied = prob_occ * est_qual;
    _unknown = 1 - _occupied;
  } else {
    _empty = (1 - prob_occ) * est_qual;
    _unknown = 1 - _empty;
  }
  */
}

#endif
