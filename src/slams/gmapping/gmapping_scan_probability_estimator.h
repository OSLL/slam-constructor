#ifndef SLAM_CTOR_GMAPPING_SCAN_PROBABILITY_ESTIMATOR_H
#define SLAM_CTOR_GMAPPING_SCAN_PROBABILITY_ESTIMATOR_H

#include <cmath>
#include <limits>
#include <memory>
#include <utility>

#include "../../core/geometry_utils.h"
#include "../../core/scan_matchers/grid_scan_matcher.h"
#include "../../core/maps/grid_map.h"

#include "gmapping_grid_cell.h"

class GmappingScanProbabilityEstimator : public ScanProbabilityEstimator {
private:
  constexpr static double FULLNESS_TH = 0.5;
  constexpr static double SIGMA_SQ = 0.01;
  constexpr static double FREE_CELL_DIST = std::sqrt(2.0);
  constexpr static double DBL_INF = std::numeric_limits<double>::infinity();
public:
  // FIXME: implement gmapping-specific OOPE (decoupling)
  GmappingScanProbabilityEstimator(OOPE oope = OOPE{nullptr})
    : ScanProbabilityEstimator{oope}
    , _scan_margin(0), _pts_skip_rate(3), _window_sz(1) {}

  double estimate_scan_probability(const LaserScan2D &scan,
                                   const RobotPose &pose,
                                   const GridMap &map,
                                   const SPEParams &) const override {
    auto sp_observation = AreaOccupancyObservation{true, {1.0, 1.0},
                                                   {0, 0}, 1.0};

    double last_dpoint_prob = -1;
    DiscretePoint2D last_handled_dpoint;
    scan.trig_provider->set_base_angle(pose.theta);

    double total_probability = 0;
    double handled_pts_nm = 0;
    auto end_point_i = scan.points().size() - _scan_margin;
    for (size_t i = _scan_margin; i < end_point_i; ++i) {
      if (_pts_skip_rate && i % _pts_skip_rate) {
        continue;
      }

      handled_pts_nm += 1;
      auto &sp = scan.points()[i];
      double c = scan.trig_provider->cos(sp.angle());
      double s = scan.trig_provider->sin(sp.angle());
      sp_observation.obstacle = {pose.x + sp.range() * c,
                                 pose.y + sp.range() * s};

      auto sp_coord = map.world_to_cell(sp_observation.obstacle);
      if (sp_coord == last_handled_dpoint && last_dpoint_prob != -1) {
        total_probability += last_dpoint_prob;
        continue;
      }

      last_handled_dpoint = sp_coord;
      double best_dist = DBL_INF;
      double d_free_x = FREE_CELL_DIST * c; // 0< int cast - implicit floor
      double d_free_y = FREE_CELL_DIST * s;
      for (int d_x = -_window_sz; d_x <= _window_sz; ++d_x) {
        for (int d_y = -_window_sz; d_y <= _window_sz; ++d_y) {
          DiscretePoint2D cell_coord = sp_coord + DiscretePoint2D(d_x, d_y);
          const GridCell &cell = map[cell_coord];
          if (cell < FULLNESS_TH) {
            continue; // cell is not occupied
          }
          cell_coord.x -= d_free_x;
          cell_coord.y -= d_free_y;
          const GridCell &free_cell = map[cell_coord];

          if (FULLNESS_TH <= free_cell) {
            continue; //occlusion is detected
          }

          double dist = cell.discrepancy(sp_observation);
          if (dist < best_dist) {
            best_dist = dist;
          }
        }
      }
      double dist_sq = best_dist != DBL_INF ? best_dist : 9 * SIGMA_SQ;
      last_dpoint_prob = exp(-dist_sq / SIGMA_SQ);
      total_probability += last_dpoint_prob;
    }

    if (handled_pts_nm == 0) { return unknown_probability(); }
    return total_probability / handled_pts_nm;
  }

private:
  int _scan_margin, _pts_skip_rate, _window_sz;
};

#endif
