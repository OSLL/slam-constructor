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
  double probability(const AreaOccupancyObservation &aoo,
                     const LightWeightRectangle &,
                     const GridMap &map) const override {
    assert(aoo.is_occupied);
    double prob = 1.0 - map[map.world_to_cell(aoo.obstacle)].discrepancy(aoo);
    assert(0 <= prob);
    return prob;
  }
};

class MaxOccupancyObservationPE
  : public OccupancyObservationProbabilityEstimator {
public:
  double probability(const AreaOccupancyObservation &aoo,
                     const LightWeightRectangle &area,
                     const GridMap &map) const override {
    assert(aoo.is_occupied);
    auto max_probability = double{0};
    auto area_ids = GridRasterizedRectangle{map, area};

    while (area_ids.has_next()) {
      auto area_id = area_ids.next();
      double obs_prob = 1.0 - map[area_id].discrepancy(aoo);
      assert(0 <= obs_prob);
      max_probability = std::max(obs_prob, max_probability);
    }
    return max_probability;
  }
};

class MeanOccupancyObservationPE
  : public OccupancyObservationProbabilityEstimator {
public:
  double probability(const AreaOccupancyObservation &aoo,
                     const LightWeightRectangle &area,
                     const GridMap &map) const override {
    assert(aoo.is_occupied);
    auto tot_probability = double{0};
    auto area_nm = unsigned{0};

    auto area_ids = GridRasterizedRectangle{map, area};
    while (area_ids.has_next()) {
      auto obs_prob = 1.0 - map[area_ids.next()].discrepancy(aoo);
      assert(0 <= obs_prob);
      tot_probability += obs_prob;
      area_nm += 1;
    }
    return area_nm ? tot_probability / area_nm : 0.5;
  }
};

class OverlapWeightedOccupancyObservationPE
  : public OccupancyObservationProbabilityEstimator {
public:
  double probability(const AreaOccupancyObservation &aoo,
                     const LightWeightRectangle &area,
                     const GridMap &map) const override {
    assert(aoo.is_occupied);
    double tot_probability = 0;
    double tot_weight = 0;

    auto area_ids = GridRasterizedRectangle{map, area};
    while (area_ids.has_next()) {
      auto area_id = area_ids.next();
      auto obs_prob = 1.0 - map[area_id].discrepancy(aoo);
      assert(0 <= obs_prob);
      auto weight = area.overlap(map.world_cell_bounds(area_id));
      tot_probability += obs_prob * weight;
      tot_weight += weight;
    }
    return tot_weight ? tot_probability / tot_weight : 0.5;
  }
};

#endif
