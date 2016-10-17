#ifndef __GMAPPING_COST_ESTIMATOR
#define __GMAPPING_COST_ESTIMATOR

#include <cmath>
#include "../core/geometry_utils.h"
#include "../core/grid_scan_matcher.h"
#include "gmapping_world.h"

class GmappingCostEstimator : public ScanCostEstimator {
private:
  constexpr static double FULLNESS_TH = 0.5;
  constexpr static double SIGMA_SQ = 0.01;
  constexpr static double FREE_CELL_DIST = std::sqrt(2.0);
public:
  GmappingCostEstimator() : _scan_margin(0), _pts_skip_rate(0), _window_sz(1) {}

  virtual double estimate_scan_cost(const RobotPose &pose,
                                    const TransformedLaserScan &scan,
                                    const GridMap &map,
                                    double min_cost = 0) {
    double scan_weight = 0;

    for (size_t i = _scan_margin; i < scan.points.size() - _scan_margin; ++i) {
      if (_pts_skip_rate && i % _pts_skip_rate) {
        continue;
      }

      const ScanPoint &sp = scan.points[i];
      double c = std::cos(sp.angle + pose.theta);
      double s = std::sin(sp.angle + pose.theta);
      Point2D sp_world(pose.x + sp.range * c, pose.y + sp.range * s);

      DiscretePoint2D sp_coord = map.world_to_cell(sp_world.x, sp_world.y);
      DiscretePoint2D free_cell_offset =
        -DiscretePoint2D(round(FREE_CELL_DIST * c), round(FREE_CELL_DIST * s));

      double best_dist = -1.0;
      for (int d_x = -_window_sz; d_x <= _window_sz; ++d_x) {
        for (int d_y = -_window_sz; d_y <= _window_sz; ++d_y) {
          DiscretePoint2D cell = sp_coord + DiscretePoint2D(d_x, d_y);
          const GridCellValue &cell_value = map[cell];
          const GridCellValue &free_cell_value = map[cell + free_cell_offset];

          if (!(free_cell_value < FULLNESS_TH && FULLNESS_TH <= cell_value)) {
            continue;
          }

          const GmappingCellValue &gmg_val =
            dynamic_cast<const GmappingCellValue&>(cell_value);

          double dist = sp_world.dist_sq(gmg_val.obst);
          if (dist < best_dist || best_dist < 0) {
            best_dist = dist;
          }
        }
      }
      if (0 < best_dist) {
        scan_weight += exp(-1.0 / SIGMA_SQ * best_dist);
      }
    }

    return scan_weight;
  }

private:
  int _scan_margin, _pts_skip_rate, _window_sz;
};

#endif
