#ifndef SLAM_CTOR_SLAMS_VINY_SLAM_H
#define SLAM_CTOR_SLAMS_VINY_SLAM_H

#include <memory>

#include "../../utils/init_scan_matching.h"
#include "../../utils/init_occupancy_mapping.h"

#include "../../core/maps/plain_grid_map.h"
#include "../../core/states/single_state_hypothesis_laser_scan_grid_world.h"

#include "viny_grid_cell.h"

using VinySlam= SingleStateHypothesisLaserScanGridWorld<UnboundedPlainGridMap>;

auto init_viny_slam(const PropertiesProvider &props) {
  auto slam_props = SingleStateHypothesisLSGWProperties{};
  // FIXME: move to params
  slam_props.localized_scan_quality = 0.9;
  slam_props.raw_scan_quality = 0.6;
  slam_props.cell_prototype = std::make_shared<VinyDSCell>();

  slam_props.gsm = init_scan_matcher(props);
  slam_props.gmsa = init_scan_adder(props);
  slam_props.map_props = init_grid_map_params(props);
  return std::make_shared<VinySlam>(slam_props);
}

#endif
