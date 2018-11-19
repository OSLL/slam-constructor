#ifndef SLAM_CTOR_SLAMS_CREDIBILIST_SLAM_H
#define SLAM_CTOR_SLAMS_CREDIBILIST_SLAM_H

#include <memory>

#include "../../utils/init_scan_matching.h"
#include "../../utils/init_occupancy_mapping.h"

#include "../../core/states/single_state_hypothesis_laser_scan_grid_world.h"

#include "grid_cell.h"

auto init_credibilist_slam(const PropertiesProvider &props) {
  auto slam_props = SingleStateHypothesisLSGWProperties{};
  // FIXME: move to params
  slam_props.localized_scan_quality = 0.9;
  slam_props.raw_scan_quality = 0.6;

  auto area_model = std::make_shared<CredibilistCell>();
  slam_props.grid_map = init_grid_map(props, area_model);
  slam_props.gsm = init_scan_matcher(props);
  slam_props.gmsa = init_scan_adder(props);
  return std::make_shared<SingleStateHypothesisLaserScanGridWorld>(slam_props);
}

#endif
