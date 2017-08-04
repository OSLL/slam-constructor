#ifndef SLAM_CTOR_SLAMS_TINY_SLAM_H
#define SLAM_CTOR_SLAMS_TINY_SLAM_H

#include <iostream>
#include <memory>

#include "../../utils/init_scan_matching.h"
#include "../../utils/init_occupancy_mapping.h"

#include "../../core/maps/plain_grid_map.h"
#include "../../core/states/single_state_hypothesis_laser_scan_grid_world.h"
#include "../../core/scan_matchers/weighted_mean_discrepancy_spe.h"

#include "tiny_grid_cell.h"

using TinySlam= SingleStateHypothesisLaserScanGridWorld<UnboundedPlainGridMap>;

void setup_tiny_cell_prototype(std::shared_ptr<PropertiesProvider> props,
                               SingleStateHypothesisLSGWProperties &sg_props) {
  auto cell_type = props->get_str("slam/cell/type", "avg");

  if (cell_type == "base") {
    // FIXME: move to params
    sg_props.localized_scan_quality = 0.2;
    sg_props.raw_scan_quality = 0.1;
    sg_props.cell_prototype = std::make_shared<BaseTinyCell>();
  } else if (cell_type == "avg") {
    // FIXME: move to params
    sg_props.localized_scan_quality = 0.9;
    sg_props.raw_scan_quality = 0.6;
    sg_props.cell_prototype = std::make_shared<AvgTinyCell>();
  } else {
    std::cerr << "Unknown cell type: " << cell_type << std::endl;
    std::exit(-1);
  }
}

// FIXME: ~code duplication init_viny_slam.cpp
auto init_tiny_slam(std::shared_ptr<PropertiesProvider> props) {
  auto slam_props = SingleStateHypothesisLSGWProperties{};
  setup_tiny_cell_prototype(props, slam_props);

  auto spe = std::make_shared<WeightedMeanDiscrepancySPEstimator>
    (init_oope(props));
  slam_props.gsm = init_scan_matcher(props, spe);

  // TODO: move to init utils, rename to map distorsion
  slam_props.gmsa = std::make_shared<WallDistanceBlurringScanAdder>(
    init_occ_estimator(props), init_mapping_blur(props));
  slam_props.map_props = init_grid_map_params(props);
  return std::make_shared<TinySlam>(slam_props);
}

#endif
