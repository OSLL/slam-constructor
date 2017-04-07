#ifndef SLAM_CTOR_CORE_MANU_TO_MANY_MULTIRES_SCAN_MANCHR_H_INCLUDED
#define SLAM_CTOR_CORE_MANU_TO_MANY_MULTIRES_SCAN_MANCHR_H_INCLUDED

#include <limits>
#include <queue>

#include "grid_scan_matcher.h"
#include "../maps/zoomable_grid_map.h"
#include "../geometry_primitives.h"

// TODO: rm this after fix
#include "../maps/plain_grid_map.h"

class ManyToManyMultiResoultionScanMatcher : public GridScanMatcher {
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

    bool operator>(const BranchAndBoundNode &rhs) const {
      return cost > rhs.cost;
    }
  };

  template <typename T>
  using max_pq = std::priority_queue<T, std::vector<T>, std::greater<T>>;

public:
  ManyToManyMultiResoultionScanMatcher(std::shared_ptr<ScanCostEstimator> est,
                                       double ang_step = deg2rad(0.05),
                                       double ang_range = deg2rad(5),
                                       double translation_range_x = 1,
                                       double translation_range_y = 1)
    : GridScanMatcher(est)
    , _ang_step{ang_step}, _ang_range{ang_range}
    , _translation_range_x{translation_range_x}
    , _translation_range_y{translation_range_y} {}

  virtual double process_scan(const RobotPose &init_pose,
                              const TransformedLaserScan &scan,
                              const GridMap &map,
                              RobotPoseDelta &result_pose_delta) {
    auto sce = GridScanMatcher::cost_estimator();
    // TODO: complete type of ZoomableGridMap; The SM should be templatazied?
    // TODO: casts
    // TODO: restore zoom level
    auto &z_map = dynamic_cast<ZoomableGridMap<UnboundedPlainGridMap> &>(
                    const_cast<GridMap&>(map));
    auto unchecked_poses = max_pq<BranchAndBoundNode>{};
    // TODO: Rectangle -> pose correction when best area is found
    // add explicit "no correction" entry
    z_map.set_zoom_level(0);
    unchecked_poses.emplace(sce->estimate_scan_cost(init_pose, scan, z_map),
                            0, Rectangle{0, 0, 0, 0}, 0);

    // init unchecked poses pool
    // TODO: dynamic init zoom level calculation
    //       (log(max_transl_window/map0.scale())
    z_map.set_zoom_level(z_map.zoom_levels_nm() - 1);
    auto pose_delta = RobotPoseDelta{0, 0, 0};
    for (double d_th = -_ang_range/2; d_th <= _ang_range/2; d_th += _ang_step) {
      pose_delta.theta = d_th;
      unchecked_poses.emplace(
        /* cost */
        sce->estimate_scan_cost(init_pose + pose_delta, scan, z_map),
        /* scan rotation */
        d_th,
        /* translation lookup area */
        Rectangle{- _translation_range_y / 2,
                  + _translation_range_y / 2,
                  - _translation_range_x / 2,
                  + _translation_range_x / 2},
         /* min resolution */
        z_map.zoom_levels_nm() - 1
      );
      /*
      std::cout << d_th << " -> "
                << sce->estimate_scan_cost(init_pose + pose_delta, scan, z_map)
                << std::endl;
      */
    }

    //std::cout << "INIT is DONE" << std::endl;
    // the best pose lookup
    while (!unchecked_poses.empty()) {
      auto best_node = unchecked_poses.top();
      auto transl_step = best_node.delta_window.side() / 2;
      bool node_is_leaf = best_node.lvl ==
                          ZoomableGridMap<GridMap>::finest_zoom_level();

      /*
      std::cout << best_node.lvl << ": " << best_node.delta_window;
      std::cout << ", " << best_node.theta;
      std::cout << " -> " << best_node.cost << std::endl;
      */
      if (node_is_leaf) {
        // search is done
        result_pose_delta = RobotPoseDelta{
          best_node.delta_window.left() + transl_step,
          best_node.delta_window.bot() + transl_step,
          best_node.theta};
        return best_node.cost;
      } else {
        // branching
        unchecked_poses.pop();
        z_map.set_zoom_level(best_node.lvl - 1);
        auto pose_delta = RobotPoseDelta{0, 0, best_node.theta};
        for (int d_x_i = 0; d_x_i < 2; ++d_x_i) {
          for (int d_y_i = 0; d_y_i < 2; ++d_y_i) {
            // TODO: move to a separate strategy (e.g. node's input iterator)
            auto srch_area = Rectangle{
              best_node.delta_window.bot() + ((d_y_i % 2 == 0) ? 0
                                                               : transl_step),
              (d_y_i % 2 == 0) ? best_node.delta_window.bot() + transl_step
                               : best_node.delta_window.top(),
              best_node.delta_window.left() + ((d_x_i % 2 == 0) ? 0
                                                                : transl_step),
              (d_x_i % 2 == 0) ? best_node.delta_window.left() + transl_step
                               : best_node.delta_window.right(),
            };
            pose_delta.x = srch_area.left() + srch_area.side() / 2;
            pose_delta.y = srch_area.bot() + srch_area.side() / 2;
            unchecked_poses.emplace(
              sce->estimate_scan_cost(init_pose + pose_delta, scan, z_map),
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
private:
  double _ang_step, _ang_range;
  double _translation_range_x, _translation_range_y;
};

#endif // include guard
