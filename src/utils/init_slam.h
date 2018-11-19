#ifndef SLAM_CTOR_UTILS_INIT_SLAM_H
#define SLAM_CTOR_UTILS_INIT_SLAM_H

#include <memory>
#include <tuple>

#include "init_scan_matching.h"
#include "init_occupancy_mapping.h"

#include "../core/states/single_state_hypothesis_laser_scan_grid_world.h"

auto init_1h_slam(const PropertiesProvider &props) {
  auto slam_props = SingleStateHypothesisLSGWProperties{};
  double loc, raw;
  std::tie(loc, raw) = init_pose_quality_estimators(props);
  slam_props.localized_scan_quality = loc;
  slam_props.raw_scan_quality = raw;

  slam_props.grid_map = init_grid_map(props);
  slam_props.gsm = init_scan_matcher(props);
  slam_props.gmsa = init_scan_adder(props);

  return std::make_shared<SingleStateHypothesisLaserScanGridWorld>(slam_props);
}

#endif
