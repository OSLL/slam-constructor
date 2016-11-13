#ifndef __STATE_DATA_H_INCLUDED
#define __STATE_DATA_H_INCLUDED

#include "geometry_utils.h"

struct Occupancy {
  double prob_occ;
  double estimation_quality;

  // TODO: is NaN better for as a dflt value?
  Occupancy(double prob = 0, double quality = 0) :
    prob_occ(prob), estimation_quality(quality) {}

  operator double() const { return prob_occ; }

  bool operator==(const Occupancy &that) {
    return EQ_DOUBLE(prob_occ, that.prob_occ) &&
           EQ_DOUBLE(estimation_quality, that.estimation_quality);
  }
};


#endif
