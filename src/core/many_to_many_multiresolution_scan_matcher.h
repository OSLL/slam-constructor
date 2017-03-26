#ifndef SLAM_CTOR_CORE_MANU_TO_MANY_MULTIRESOLUTION_SCAN_MANCHR_H_INCLUDED
#define SLAM_CTOR_CORE_MANU_TO_MANY_MULTIRESOLUTION_SCAN_MANCHR_H_INCLUDED

#include <limits>
#include <queue>

#include "grid_scan_matcher.h"
#include "maps/zoomable_grid_map.h"
#include "geometry_primitives.h"

// TODO: rm this after fix
#include "maps/plain_grid_map.h"

class ManyToManyMultiResoultionScanMatcher : public GridScanMatcher {
private:
  static constexpr double D_Th = 0.1 * M_PI / 180;
  static constexpr double Half_Th_Srch = 15 * M_PI / 180;
  static constexpr double Half_Transl_Srch = 1;
private:
  struct BranchAndBoundNode {
    double cost;
    double theta;
    Rectangle delta_window;
    int lvl;

    BranchAndBoundNode(double c, double th, const Rectangle& wn, int l)
      : cost{c}, theta{th}, delta_window{wn}, lvl{l} {}
    BranchAndBoundNode(const BranchAndBoundNode&) = default;
    BranchAndBoundNode(BranchAndBoundNode&&) = default;
    BranchAndBoundNode& operator=(const BranchAndBoundNode&) = default;
    BranchAndBoundNode& operator=(BranchAndBoundNode&&) = default;

    bool operator<(const BranchAndBoundNode &rhs) const {
      return cost < rhs.cost;
    }
  };
public:
  ManyToManyMultiResoultionScanMatcher(std::shared_ptr<ScanCostEstimator> est)
    : GridScanMatcher(est) {}

  virtual double process_scan(const RobotPose &init_pose,
                              const TransformedLaserScan &scan,
                              const GridMap &map,
                              RobotPoseDelta &result_pose_delta) {
    auto sce = GridScanMatcher::cost_estimator();
    // TODO: complete type of ZoomableGridMap
    auto &z_map = dynamic_cast<ZoomableGridMap<UnboundedPlainGridMap> &>(
                    const_cast<GridMap&>(map));
    auto unchecked_poses = std::priority_queue<BranchAndBoundNode>{};

    z_map.set_zoom_level(z_map.zoom_levels_nm() - 1);
    auto pose_delta = RobotPoseDelta{0, 0, 0};
    for (double d_th = -Half_Th_Srch; d_th < Half_Th_Srch; d_th += D_Th) {
      pose_delta.theta = d_th;
      Rectangle r1, r2;
      r1 = r2;
      unchecked_poses.emplace(
        /* cost */
        sce->estimate_scan_cost(init_pose + pose_delta, scan, z_map,
                                std::numeric_limits<double>::max()),
        /* scan rotation */
        init_pose.theta + d_th,
        /* translation lookup area */
        Rectangle{-Half_Transl_Srch, Half_Transl_Srch,
                  -Half_Transl_Srch, Half_Transl_Srch},
         /* min resolution */
        z_map.zoom_levels_nm() - 1
      );
    }

    while (!unchecked_poses.empty()) {
      auto best_node = unchecked_poses.top();
      auto transl_step = best_node.delta_window.side() / 2;
      bool node_is_leaf = best_node.lvl ==
                          ZoomableGridMap<GridMap>::Unzoomed_Map_Level;
      if (node_is_leaf) {
        // search is done
        result_pose_delta = RobotPoseDelta{transl_step, transl_step,
                                           best_node.theta};
        std::cout << result_pose_delta << std::endl;
        return best_node.cost;
      } else {
        unchecked_poses.pop();
        z_map.set_zoom_level(best_node.lvl-1);
        auto pose_delta = RobotPoseDelta{0, 0, best_node.theta};
        for (int d_x_i = 0; d_x_i < 2; ++d_x_i) {
          for (int d_y_i = 0; d_y_i < 2; ++d_y_i) {
            auto srch_area = Rectangle{
              best_node.delta_window.bot() + ((d_y_i % 2 == 0) ? 0
                                                               : transl_step),
              (d_y_i % 2 == 0) ? best_node.delta_window.bot() + transl_step
                               : best_node.delta_window.top(),
              best_node.delta_window.left() +
                ((d_x_i % 2 == 0) ? 0 : transl_step),
              (d_x_i % 2 == 0) ? best_node.delta_window.left() + transl_step
                               : best_node.delta_window.right(),
            };
            pose_delta.y = pose_delta.x = srch_area.side() / 2;
            unchecked_poses.emplace(
              sce->estimate_scan_cost(init_pose + pose_delta, scan, z_map,
                                      std::numeric_limits<double>::max()),
              best_node.theta,
              srch_area,
              best_node.lvl - 1
            );
          }
        }
      } // if (node_is_leaf
    }
    // BUG: no pose delta has been found
    return std::numeric_limits<double>::quiet_NaN();
  }

};

#endif // include guard
