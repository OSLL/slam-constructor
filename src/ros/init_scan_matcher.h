#ifndef SLAM_CTOR_ROS_INIT_SCAN_MATCHER_H
#define SLAM_CTOR_ROS_INIT_SCAN_MATCHER_H

#include <string>
#include <memory>

#include <ros/ros.h>

#include "../core/scan_matchers/monte_carlo_scan_matcher.h"
#include "../core/scan_matchers/hill_climbing_scan_matcher.h"
#include "../core/scan_matchers/brute_force_scan_matcher.h"

auto init_monte_carlo_sm(std::shared_ptr<ScanProbabilityEstimator> spe) {
  using ros::param::param;
  static const std::string Param_Pfx = "~slam/scmtch/MC/";

  double transl_dispersion, rot_dispersion;
  int failed_attempts_per_dispersion_limit, attempts_limit, seed;
  param<double>(Param_Pfx + "dispersion/translation", transl_dispersion, 0.2);
  param<double>(Param_Pfx + "dispersion/rotation", rot_dispersion, 0.1);
  param<int>(Param_Pfx + "dispersion/failed_attempts_limit",
             failed_attempts_per_dispersion_limit, 20);
  param<int>(Param_Pfx + "attempts_limit", attempts_limit, 100);
  param<int>(Param_Pfx + "seed", seed, std::random_device{}());
  assert(0 <= failed_attempts_per_dispersion_limit && 0 <= attempts_limit);

  ROS_INFO("MC Scan Matcher seed: %u\n", seed);
  return std::make_shared<MonteCarloScanMatcher>(
      spe, seed, transl_dispersion, rot_dispersion,
      failed_attempts_per_dispersion_limit, attempts_limit);
}

auto init_hill_climbing_sm(std::shared_ptr<ScanProbabilityEstimator> spe) {
  using ros::param::param;
  static const std::string Param_Pfx = "~slam/scmtch/HC/";

  double transl_distorsion, rot_distorsion;
  int max_lu_attempts_failed;
  param<double>(Param_Pfx + "distorsion/translation", transl_distorsion, 0.1);
  param<double>(Param_Pfx + "distorsion/rotation", rot_distorsion, 0.1);
  param<int>(Param_Pfx + "lookup_attempts_failed", max_lu_attempts_failed, 6);
  assert(0 <= max_lu_attempts_failed);

  return std::make_shared<HillClimbingScanMatcher>(
      spe, max_lu_attempts_failed, transl_distorsion, rot_distorsion);
}

auto init_brute_force_sm(std::shared_ptr<ScanProbabilityEstimator> spe) {
  using ros::param::param;
  static const std::string BF_Param_Prefix = "~slam/scmtch/BF/";

  #define INIT_BFSM_RANGE(dim, limit, step)                                \
    double from_##dim, to_##dim, step_##dim;                               \
    param<double>(BF_Param_Prefix + #dim + "/from", from_##dim, -(limit)); \
    param<double>(BF_Param_Prefix + #dim + "/to"  , to_##dim  , limit);    \
    param<double>(BF_Param_Prefix + #dim + "/step", step_##dim, step);

  INIT_BFSM_RANGE(x, 0.5, 0.1);
  INIT_BFSM_RANGE(y, 0.5, 0.1);
  INIT_BFSM_RANGE(t, deg2rad(5), deg2rad(1));

  #undef INIT_BFSM_RANGE

  return std::make_shared<BruteForceScanMatcher>(
    spe, from_x, to_x, step_x, from_y, to_y, step_y, from_t, to_t, step_t);
}

std::shared_ptr<GridScanMatcher> init_scan_matcher(
    std::shared_ptr<ScanProbabilityEstimator> spe) {
  using ros::param::param;
  std::string sm_type;

  param<std::string>("~slam/scmtch/type", sm_type, "MC");
  if      (sm_type == "MC") { return init_monte_carlo_sm(spe); }
  else if (sm_type == "HC") { return init_hill_climbing_sm(spe); }
  else if (sm_type == "BF") { return init_brute_force_sm(spe); }
  else {
    std::cerr << "Unknown scan matcher type: " << sm_type << std::endl;
    std::exit(-1);
  }
}

#endif
