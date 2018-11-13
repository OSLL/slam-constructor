#ifndef SLAM_CTOR_CORE_OBSERVATION_IMPACT_ESTIMATORS_H
#define SLAM_CTOR_CORE_OBSERVATION_IMPACT_ESTIMATORS_H

#include "grid_scan_matcher.h"

class MockOIE : public ObservationImpactEstimator {
public:
  double estimate_impact(const GridCell &,
                         const AreaOccupancyObservation &) const override {
    return -1.0;
  }
};

class DiscrepancyOIE : public ObservationImpactEstimator {
public:
  double estimate_impact(const GridCell &area,
                         const AreaOccupancyObservation &aoo) const override {
    return 1.0 - area.discrepancy(aoo);
  }
};

class OccupancyOIE  : public ObservationImpactEstimator {
public:
  double estimate_impact(const GridCell &area,
                         const AreaOccupancyObservation &aoo) const override {
    return double(area.occupancy());
  }
};

#endif
