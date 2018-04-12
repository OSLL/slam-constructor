#ifndef SLAM_CTOR_UTILS_INIT_SCAN_MATCHING_H
#define SLAM_CTOR_UTILS_INIT_SCAN_MATCHING_H

#include <string>
#include <memory>
#include <iostream>

#include "properties_providers.h"

#include "../core/scan_matchers/occupancy_observation_probability.h"

#include "../core/scan_matchers/monte_carlo_scan_matcher.h"
#include "../core/scan_matchers/hill_climbing_scan_matcher.h"
#include "../core/scan_matchers/hcsm_fixed.h"
#include "../core/scan_matchers/brute_force_scan_matcher.h"
#include "../core/scan_matchers/connect_the_dots_ambiguous_drift_detector.h"
#include "../core/scan_matchers/weighted_mean_point_probability_spe.h"

static const std::string Slam_SM_NS = "slam/scmtch/";

/*============================================================================*/
/* Init Occupancy Obeservation Probability Estimator                          */

std::shared_ptr<OccupancyObservationProbabilityEstimator> init_oope(
    const PropertiesProvider &props) {
  static const std::string OOPE_NS = Slam_SM_NS + "oope/";
  auto type = props.get_str(OOPE_NS + "type", "obstacle");

  if (type == "obstacle") {
    return std::make_shared<ObstacleBasedOccupancyObservationPE>();
  } else if (type == "max") {
    return std::make_shared<MaxOccupancyObservationPE>();
  } else if (type == "mean") {
    return std::make_shared<MeanOccupancyObservationPE>();
  } else if (type == "overlap") {
    return std::make_shared<OverlapWeightedOccupancyObservationPE>();
  } else {
    std::cerr << "Unknown occupancy observation probability estimator type "
              << "(" << OOPE_NS + "type) " << type << std::endl;
    std::exit(-1);
  }
}

/*============================================================================*/
/*                 Scan Probability Estimator initialization                  */

auto init_swp(const PropertiesProvider &props) {
  static const std::string SWP_Type = Slam_SM_NS + "spe/wmpp/weighting/type";
  auto type = props.get_str(SWP_Type, "<undefined>");
  auto swp = std::shared_ptr<ScanPointWeighting>{};

  std::cout << "Used SWP: " << type << std::endl;
  if (type == "even") {
    swp = std::make_shared<EvenSPW>();
  } else if (type == "viny") {
    swp = std::make_shared<VinySlamSPW>();
  } else if (type == "ahr") {
    swp = std::make_shared<AngleHistogramReciprocalSPW>();
  } else {
    std::cerr << "Unknown Scan Point Weighting type "
              << "(" << SWP_Type << ") " << type << std::endl;
    std::exit(-1);
  }
  return swp;
}

auto init_spe(const PropertiesProvider &props,
              std::shared_ptr<OccupancyObservationProbabilityEstimator> oope) {
  auto type = props.get_str(Slam_SM_NS + "spe/type", "<undefined>");
  if (type == "wmpp") {
    const std::string WMPP_Prefix = Slam_SM_NS + "spe/wmpp";
    auto skip_rate = props.get_uint(WMPP_Prefix + "/sp_skip_rate", 0);
    auto max_range = props.get_dbl(WMPP_Prefix + "/sp_max_usable_range", -1);
    using WmppSpe = WeightedMeanPointProbabilitySPE;
    return std::make_shared<WmppSpe>(oope, init_swp(props),
                                     skip_rate, max_range);
  } else {
    std::cerr << "Unknown Scan Probability Estimator type ("
              << Slam_SM_NS << "spe/type): " << type << std::endl;
    std::exit(-1);
  }
};

auto init_spe(const PropertiesProvider &props) {
  return init_spe(props, init_oope(props));
};

/*============================================================================*/
/* Init Scan Matcher Algorithm                                                */

auto init_monte_carlo_sm(const PropertiesProvider &props,
                         std::shared_ptr<ScanProbabilityEstimator> spe) {
  static const std::string SM_NS = Slam_SM_NS + "MC/";
  static const std::string DISP_NS = SM_NS + "dispersion/";

  auto transl_dispersion = props.get_dbl(DISP_NS + "translation", 0.2);
  auto rot_dispersion = props.get_dbl(DISP_NS + "rotation", 0.1);
  auto failed_attempts_per_dispersion_limit =
    props.get_uint(DISP_NS + "failed_attempts_limit", 20);

  auto attempts_limit = props.get_uint(SM_NS + "attempts_limit", 100);
  auto seed = props.get_int(SM_NS + "seed", std::random_device{}());

  std::cout << "[INFO] MC Scan Matcher seed: " << seed << std::endl;
  return std::make_shared<MonteCarloScanMatcher>(
      spe, seed, transl_dispersion, rot_dispersion,
      failed_attempts_per_dispersion_limit, attempts_limit);
}

auto init_hill_climbing_sm(const PropertiesProvider &props,
                           std::shared_ptr<ScanProbabilityEstimator> spe) {
  static const std::string SM_NS = Slam_SM_NS + "HC/";
  static const std::string DIST_NS = SM_NS + "distortion/";

  auto transl_distorsion = props.get_dbl(DIST_NS + "translation", 0.1);
  auto rot_distorsion = props.get_dbl(DIST_NS + "rotation", 0.1);
  auto fal = props.get_uint(DIST_NS + "failed_attempts_limit", 6);

  return std::make_shared<HillClimbingScanMatcher>(
      spe, fal, transl_distorsion, rot_distorsion);
}

auto init_brute_force_sm(const PropertiesProvider &props,
                         std::shared_ptr<ScanProbabilityEstimator> spe) {
  static const std::string SM_NS = Slam_SM_NS + "BF/";

  #define INIT_BFSM_RANGE(dim, limit, step)                             \
    auto from_##dim = props.get_dbl(SM_NS + #dim + "/from", -(limit)); \
    auto to_##dim = props.get_dbl(SM_NS + #dim + "/to", limit);        \
    auto step_##dim = props.get_dbl(SM_NS + #dim + "/step", step);

  INIT_BFSM_RANGE(x, 0.5, 0.1);
  INIT_BFSM_RANGE(y, 0.5, 0.1);
  INIT_BFSM_RANGE(t, deg2rad(5), deg2rad(1));

  #undef INIT_BFSM_RANGE

  return std::make_shared<BruteForceScanMatcher>(
    spe, from_x, to_x, step_x, from_y, to_y, step_y, from_t, to_t, step_t);
}

auto init_scan_matcher(const PropertiesProvider &props) {
  auto spe = init_spe(props);
  auto sm = std::shared_ptr<GridScanMatcher>{};
  auto sm_type = props.get_str(Slam_SM_NS + "type", "<undefined>");
  std::cout << "Used Scan Matcher: " << sm_type << std::endl;
  if      (sm_type == "MC") { sm = init_monte_carlo_sm(props, spe); }
  else if (sm_type == "HC") { sm = init_hill_climbing_sm(props, spe); }
  else if (sm_type == "BF") { sm = init_brute_force_sm(props, spe); }
  else if (sm_type == "HC_FIXED") {
    // TODO: params setup
    sm = std::make_shared<HillClimbingSMFixed>(spe);
  } else {
    std::cerr << "Unknown scan matcher type: " << sm_type << std::endl;
    std::exit(-1);
  }

  // TODO: do we need AmbDD to be a wrapper?
  if (props.get_bool(Slam_SM_NS + "use_amb_drift_detector", false)) {
    sm = std::make_shared<ConnectTheDotsAmbiguousDriftDetector>(sm);
  }
  return sm;
}

#endif
