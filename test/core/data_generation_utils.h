#ifndef SLAM_CTOR_TEST_CORE_DATA_GENERATION_UTILS_H
#define SLAM_CTOR_TEST_CORE_DATA_GENERATION_UTILS_H

#include "../../src/core/math_utils.h"
#include "../../src/utils/data_generation/laser_scan_generator.h"

constexpr static auto to_lsp(double max_dist, double fow_deg, unsigned pts_nm) {
  return LaserScannerParams{max_dist,
      deg2rad(fow_deg / pts_nm), deg2rad(fow_deg / 2.0)};
}

#endif
