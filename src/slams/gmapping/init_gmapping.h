#ifndef SLAM_CTOR_SLAMS_INIT_GMAPPING_H
#define SLAM_CTOR_SLAMS_INIT_GMAPPING_H

#include "../../utils/init_scan_matching.h"
#include "../../utils/init_occupancy_mapping.h"
#include "../../core/scan_matchers/weighted_mean_point_probability_spe.h"

#include "gmapping_occupancy_observation_pe.h"
#include "gmapping_particle_filter.h"

auto init_particles_nm(const PropertiesProvider &props) {
  return props.get_uint("slam/particles/number", 30);
}

auto init_gmapping_params(const PropertiesProvider &props) {
  auto mean_sample_xy = props.get_dbl("slam/particles/sample/xy/mean", 0.0);
  auto sigma_sample_xy = props.get_dbl("slam/particles/sample/xy/sigma", 0.1);
  auto mean_sample_th = props.get_dbl("slam/particles/sample/theta/mean", 0.0);
  auto sigma_sample_th = props.get_dbl("slam/particles/sample/theta/sigma",
                                        0.03);

  auto min_sm_lim_xy = props.get_dbl("slam/particles/sm_delta_lim/xy/min",
                                      0.6);
  auto max_sm_lim_xy = props.get_dbl("slam/particles/sm_delta_lim/xy/max",
                                      0.8);
  auto min_sm_lim_th = props.get_dbl("slam/particles/sm_delta_lim/theta/min",
                                      0.3);
  auto max_sm_lim_th = props.get_dbl("slam/particles/sm_delta_lim/theta/max",
                                      0.4);
  return GMappingParams{mean_sample_xy, sigma_sample_xy,
                        mean_sample_th, sigma_sample_th,
                        min_sm_lim_xy, max_sm_lim_xy,
                        min_sm_lim_th, max_sm_lim_th};
}

auto init_gmapping_prob_estimator(const PropertiesProvider &props) {
  const std::string OOPE_Pfx = "slam/scmtch/oope/";
  assert(props.get_str(OOPE_Pfx + "type", "custom") == "custom");

  using CustomOOPE = GmappingOccupancyObservationPE;
  auto fullness_th = props.get_dbl(OOPE_Pfx + "custom/fullness_threshold", 0.1);
  auto window_size = props.get_uint(OOPE_Pfx + "cutrom/window_size", 1);
  auto oope = std::make_shared<CustomOOPE>(fullness_th, window_size);
  return init_spe(props, oope);
}

using Gmapping = GmappingParticleFilter;

auto init_gmapping(const PropertiesProvider &props) {
  // TODO: remove grid cell strategy
  auto shw_params = SingleStateHypothesisLSGWProperties{
    1.0, 1.0, 0, std::make_shared<GmappingBaseCell>(),
    // FIXME: move to params
    std::make_shared<HillClimbingScanMatcher>(
      init_gmapping_prob_estimator(props),
      6, 0.1, 0.1),
    init_scan_adder(props),
    init_grid_map_params(props)
  };
  return std::make_shared<GmappingParticleFilter>(shw_params,
    init_gmapping_params(props), init_particles_nm(props));
}

#endif
