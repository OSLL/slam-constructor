#ifndef __VINY_SCAN_COST_ESTIMATOR
#define __VINY_SCAN_COST_ESTIMATOR

#include "../core/sensor_data.h"
#include "../core/grid_scan_matcher.h"

class VinyScanCostEstimator : public ScanCostEstimator {
public:
  virtual double estimate_scan_cost(const RobotPose &pose,
                                    const TransformedLaserScan &scan,
                                    const GridMap &map,
                                    double min_cost) override {
    auto OCCUPIED_OBSERVATION = AreaOccupancyObservation{
      true, {1.0, 1.0}, {0, 0}, 1.0};
    double cost = 0;
    for (const auto &sp : scan.points) {
      if (!sp.is_occupied) {
        continue;
      }
      // move to world frame assume sensor coords (0,0)
      double x_world = pose.x + sp.range * std::cos(sp.angle+pose.theta);
      double y_world = pose.y + sp.range * std::sin(sp.angle+pose.theta);
      //double cost_weight = 1.0;
      //double cost_weight = std::abs(std::cos(sp.angle)) + 0.5;// * sp.range;
      double cost_weight = std::abs(std::sin(sp.angle)) +
                           std::abs(std::cos(sp.angle));
      if (0.9 < std::abs(std::cos(sp.angle))) {
        cost_weight = 3;
      } else if (0.8 < std::abs(std::cos(sp.angle))) {
        cost_weight = 2;
      }
      cost_weight *= std::sqrt(sp.range);
      DiscretePoint2D cell_coord = map.world_to_cell(x_world, y_world);
      if (!map.has_cell(cell_coord)) {
        cost += 1.0 * cost_weight;
        continue;
      }

      cost += map[cell_coord].discrepancy(OCCUPIED_OBSERVATION) * cost_weight;
      if (min_cost < cost) {
        break;
      }
    }
    return cost;
  }

};

#endif
