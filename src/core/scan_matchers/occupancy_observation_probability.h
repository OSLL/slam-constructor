#ifndef SLAM_CTOR_CORE_OCCUPANCY_OBSERVATION_PROBABILITY_H
#define SLAM_CTOR_CORE_OCCUPANCY_OBSERVATION_PROBABILITY_H

#include <cmath>
#include "../maps/grid_rasterization.h"
#include "grid_scan_matcher.h"

// TODO: add an option that alters
//       aoo.observation quality based on overlap
//       map.world_cell_bounds(area_id).overlap(area); // NB: order

class ObstacleBasedOccupancyObservationPE
  : public OccupancyObservationProbabilityEstimator {
public:
  ObstacleBasedOccupancyObservationPE(std::shared_ptr<OIE> oie)
    : OccupancyObservationProbabilityEstimator{oie} {}

  double probability(const AreaOccupancyObservation &aoo,
                     const LightWeightRectangle &,
                     const GridMap &map) const override {
    assert(aoo.is_occupied);
    const auto &area = map[map.world_to_cell(aoo.obstacle)];
    auto impact = observation_impact(area, aoo);
    assert(is_probality_bounded(impact));
    return impact;
  }
};

class MaxOccupancyObservationPE
  : public OccupancyObservationProbabilityEstimator {
public:
  MaxOccupancyObservationPE(std::shared_ptr<OIE> oie)
    : OccupancyObservationProbabilityEstimator{oie} {}

  double probability(const AreaOccupancyObservation &aoo,
                     const LightWeightRectangle &area,
                     const GridMap &map) const override {
    assert(aoo.is_occupied);
    auto max_probability = double{0};
    auto area_ids = GridRasterizedRectangle{map, area};

    while (area_ids.has_next()) {
      double impact = observation_impact(map[area_ids.next()], aoo);
      assert(is_probality_bounded(impact));
      max_probability = std::max(impact, max_probability);
    }
    return max_probability;
  }
};

class MeanOccupancyObservationPE
  : public OccupancyObservationProbabilityEstimator {
public:
  MeanOccupancyObservationPE(std::shared_ptr<OIE> oie)
    : OccupancyObservationProbabilityEstimator{oie} {}

  double probability(const AreaOccupancyObservation &aoo,
                     const LightWeightRectangle &area,
                     const GridMap &map) const override {
    assert(aoo.is_occupied);
    auto tot_probability = double{0};
    auto area_nm = unsigned{0};

    auto area_ids = GridRasterizedRectangle{map, area};
    while (area_ids.has_next()) {
      double impact = observation_impact(map[area_ids.next()], aoo);
      assert(is_probality_bounded(impact));
      tot_probability += impact;
      area_nm += 1;
    }
    return area_nm ? tot_probability / area_nm : 0.5;
  }
};

class OverlapWeightedOccupancyObservationPE
  : public OccupancyObservationProbabilityEstimator {
public:
  OverlapWeightedOccupancyObservationPE(std::shared_ptr<OIE> oie)
    : OccupancyObservationProbabilityEstimator{oie} {}

  double probability(const AreaOccupancyObservation &aoo,
                     const LightWeightRectangle &area,
                     const GridMap &map) const override {
    assert(aoo.is_occupied);
    double tot_probability = 0;
    double tot_weight = 0;

    auto area_ids = GridRasterizedRectangle{map, area};
    while (area_ids.has_next()) {
      auto area_id = area_ids.next();
      double impact = observation_impact(map[area_id], aoo);
      assert(is_probality_bounded(impact));
      auto weight = area.overlap(map.world_cell_bounds(area_id));
      tot_probability += impact * weight;
      tot_weight += weight;
    }
    return tot_weight ? tot_probability / tot_weight : 0.5;
  }
};

#endif
