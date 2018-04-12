#ifndef SLAM_CTOR_GMAPPING_OCCUPANCY_OBSERVATION_PE
#define SLAM_CTOR_GMAPPING_OCCUPANCY_OBSERVATION_PE

#include <cmath>
#include <cassert>

#include "../../core/scan_matchers/occupancy_observation_probability.h"
#include "../../core/geometry_utils.h"

class GmappingOccupancyObservationPE
  : public OccupancyObservationProbabilityEstimator {
public:
  GmappingOccupancyObservationPE(double fullness_th, unsigned window_size)
    : Fullness_Th{fullness_th}, Window_Sz{int(window_size)}, _cached_prob{-1} {}
  double probability(const AreaOccupancyObservation &aoo,
                     const LightWeightRectangle &,
                     const GridMap &map) const override {
    assert(aoo.is_occupied);
    auto sp_coord = map.world_to_cell(aoo.obstacle);
    if (sp_coord == _cached_coord && _cached_prob != -1) {
      return _cached_prob;
    }

    double best_prob = 0;
    for (int d_x = -Window_Sz; d_x <= Window_Sz; ++d_x) {
      for (int d_y = -Window_Sz; d_y <= Window_Sz; ++d_y) {
        auto cell_coord = sp_coord + DiscretePoint2D{d_x, d_y};
        const auto &cell = map[cell_coord];
        if (cell < Fullness_Th) { continue; }
        best_prob = std::max(best_prob, 1.0 - cell.discrepancy(aoo));
      }
    }

    _cached_coord = sp_coord;
    return _cached_prob = best_prob;
  }

private:
  const double Fullness_Th;
  const int Window_Sz;
  mutable DiscretePoint2D _cached_coord;
  mutable double _cached_prob;
};


#endif
