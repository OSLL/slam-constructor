#ifndef SLAM_CTOR_SLAMS_TINY_SLAM_H
#define SLAM_CTOR_SLAMS_TINY_SLAM_H

#include <iostream>
#include <memory>

#include "../../utils/init_scan_matching.h"
#include "../../utils/init_occupancy_mapping.h"

#include "../../core/maps/plain_grid_map.h"
#include "../../core/states/single_state_hypothesis_laser_scan_grid_world.h"

#include "tiny_grid_cell.h"

using TinySlam= SingleStateHypothesisLaserScanGridWorld<UnboundedPlainGridMap>;

void setup_tiny_cell_prototype(const PropertiesProvider &props,
                               SingleStateHypothesisLSGWProperties &sg_props) {
  auto cell_type = props.get_str("slam/cell/type", "avg");

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
auto init_tiny_slam(const PropertiesProvider &props) {
  auto slam_props = SingleStateHypothesisLSGWProperties{};
  setup_tiny_cell_prototype(props, slam_props);

  slam_props.gsm = init_scan_matcher(props);
  slam_props.gmsa = init_scan_adder(props);
  slam_props.map_props = init_grid_map_params(props);
  return std::make_shared<TinySlam>(slam_props);
}

#endif
