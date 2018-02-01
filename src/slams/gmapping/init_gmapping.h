#ifndef SLAM_CTOR_SLAMS_INIT_GMAPPING_H
#define SLAM_CTOR_SLAMS_INIT_GMAPPING_H

#include "../../utils/init_scan_matching.h"
#include "../../utils/init_occupancy_mapping.h"

#include "gmapping_scan_probability_estimator.h"
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

using Gmapping = GmappingParticleFilter;

auto init_gmapping(const PropertiesProvider &props) {
  // TODO: remove grid cell strategy
  auto shw_params = SingleStateHypothesisLSGWProperties{
    1.0, 1.0, 0, std::make_shared<GmappingBaseCell>(),
    // FIXME: move to params
    std::make_shared<HillClimbingScanMatcher>(
      std::make_shared<GmappingScanProbabilityEstimator>(),
      6, 0.1, 0.1),
    std::make_shared<WallDistanceBlurringScanAdder>(
      init_occ_estimator(props), 0
    ),
    init_grid_map_params(props)
  };
  return std::make_shared<GmappingParticleFilter>(shw_params,
    init_gmapping_params(props), init_particles_nm(props));
}

#endif
