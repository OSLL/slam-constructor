#ifndef SLAM_CTOR_CORE_MANU_TO_MANY_MULTIRES_SCAN_MANCHR_H_INCLUDED
#define SLAM_CTOR_CORE_MANU_TO_MANY_MULTIRES_SCAN_MANCHR_H_INCLUDED

#include <limits>
#include <queue>
#include <vector>

#include "grid_scan_matcher.h"
#include "../maps/zoomable_grid_map.h"
#include "../geometry_primitives.h"

class Pow2BranchingPolicy {
public:
  static std::vector<Rectangle> branch(const Rectangle &area) {
    auto result = std::vector<Rectangle>{};
    result.reserve(4);
    auto dst_hside = area.hside_len() / 2;
    auto dst_vside = area.vside_len() / 2;
    for (int d_x_i = 0; d_x_i < 2; ++d_x_i) {
      for (int d_y_i = 0; d_y_i < 2; ++d_y_i) {
        result.emplace_back(
          area.bot() + ((d_y_i % 2 == 0) ? 0 : dst_vside),
          (d_y_i % 2 == 0) ? area.bot() + dst_vside : area.top(),
          area.left() + ((d_x_i % 2 == 0) ? 0 : dst_hside),
          (d_x_i % 2 == 0) ? area.left() + dst_hside : area.right()
        );
      }
    }
    return result;
  }

  static unsigned branchings_nm(double init_sz, double target_sz) {
    // FIXME: branchings estimate calculation
    // Motivation: performance (?)
    assert(target_sz <= init_sz);
    return unsigned(std::ceil(std::log(init_sz / target_sz)));
  }
};

template<typename CoreMapType, typename BranchingPolicy = Pow2BranchingPolicy>
class ManyToManyMultiResoultionScanMatcher : public GridScanMatcher {
private: // consts
  static constexpr double _Max_Translation_Error = 0.1,
                          _Max_Rotation_Error = deg2rad(2);
private: // types
  struct BranchAndBoundNode {
    // TODO: map ptr/ref
    double cost;
    double theta;
    Rectangle translation_window;
    int lvl;

    BranchAndBoundNode(double c, double th, const Rectangle& t_wn, int l)
      : cost{c}, theta{th}, translation_window{t_wn}, lvl{l} {}
    BranchAndBoundNode(const BranchAndBoundNode&) = default;
    BranchAndBoundNode(BranchAndBoundNode&&) = default;
    BranchAndBoundNode& operator=(const BranchAndBoundNode&) = default;
    BranchAndBoundNode& operator=(BranchAndBoundNode&&) = default;

    // returns true if this node is less prepefable than a given one
    bool operator<(const BranchAndBoundNode &other) const {
      if (cost == other.cost) {
        return lvl > other.lvl; // finer is "better"
      }
      // TODO: replace cost with probability in scan matcher
      return cost > other.cost; // smaller is "better"
    }
  };

  using UncheckedPoses = std::priority_queue<BranchAndBoundNode>;
public:
  // TODO: do we need max_*_error setup in ctor?
  ManyToManyMultiResoultionScanMatcher(std::shared_ptr<ScanCostEstimator> est,
                                       double ang_step = deg2rad(0.1),
                                       double tr_error_factor = 2)
    : GridScanMatcher{est, _Max_Translation_Error, _Max_Translation_Error,
                      _Max_Rotation_Error}
    , _ang_step{ang_step} , _tr_error_factor{tr_error_factor} {}

  double process_scan(const RobotPose &init_pose,
                      const TransformedLaserScan &scan,
                      const GridMap &map,
                      RobotPoseDelta &result_pose_delta) override {
    // TODO: dynamic angle step estimate
    //std::cout << "M3RS Start" << std::endl;
    // FIXME?: dynamic cast; iterators over zoomed maps?
    auto &z_map = dynamic_cast<ZoomableGridMap<CoreMapType> &>(
                    const_cast<GridMap&>(map));
    auto unchecked_poses = init_unchecked_poses(init_pose, scan, z_map);

    auto sce = GridScanMatcher::cost_estimator();
    auto acceptable_transl_error = max_translation_error(z_map);
    // the best pose lookup
    while (!unchecked_poses.empty()) {
      auto best_node = unchecked_poses.top();
      bool node_is_leaf = best_node.lvl ==
                          ZoomableGridMap<GridMap>::finest_zoom_level();
      auto& tr_wn = best_node.translation_window;
      if (node_is_leaf) {
        // search is done
        result_pose_delta = RobotPoseDelta{tr_wn.left() + tr_wn.hside_len() / 2,
                                           tr_wn.bot() + tr_wn.vside_len() / 2,
                                           best_node.theta};
        //std::cout << "M3RS End -> " << result_pose_delta << std::endl;
        return best_node.cost;
      } else {
        // branching
        auto branched_windows = std::vector<Rectangle>{};
        auto should_branch = acceptable_transl_error < tr_wn.side();
        if (should_branch) {
          branched_windows = BranchingPolicy::branch(tr_wn);
        } else {
          branched_windows.push_back(tr_wn);
        }

        // update unchecked poses
        unchecked_poses.pop();
        z_map.set_zoom_level(best_node.lvl - 1);
        auto branch_pdelta = RobotPoseDelta{0, 0, best_node.theta};
        for (auto& window : branched_windows) {
          branch_pdelta.x = window.left() + window.hside_len() / 2;
          branch_pdelta.y = window.bot() + window.vside_len() / 2;
          auto branch_cost = sce->estimate_scan_cost(init_pose + branch_pdelta,
                                                     scan, z_map);
          unchecked_poses.emplace(branch_cost, best_node.theta, window,
                                  best_node.lvl - 1);
        }
      } // if (node_is_leaf)
    }
    // std::cout << "M3RS End ?!" << std::endl;
    // BUG: no pose delta has been found
    return std::numeric_limits<double>::quiet_NaN();
  }

private: // methods
  void setup_init_zoom_level(ZoomableGridMap<CoreMapType> &map) {
    // FIXME
    /*
    auto t_error = max_translation_error(map);
    auto x_lvl = BranchingPolicy::branchings_nm(_translation_range_x, t_error);
    auto y_lvl = BranchingPolicy::branchings_nm(_translation_range_y, t_error);
    auto zoom_lvl = std::max(x_lvl, y_lvl);
    map.set_zoom_level(std::min(zoom_lvl, map.coarsest_zoom_level()));
    */
    map.set_zoom_level(map.coarsest_zoom_level());
  }

  double max_translation_error(ZoomableGridMap<CoreMapType> &map) {
    map.set_zoom_level(map.finest_zoom_level());
    return map.scale() / _tr_error_factor;
  }

  UncheckedPoses init_unchecked_poses(const RobotPose &pose,
                                       const TransformedLaserScan &scan,
                                       ZoomableGridMap<CoreMapType> &map) {
    UncheckedPoses unchecked_poses;
    auto sce = GridScanMatcher::cost_estimator();
    // add explicit "no correction" entry
    map.set_zoom_level(map.finest_zoom_level());
    unchecked_poses.emplace(sce->estimate_scan_cost(pose, scan, map),
                            0, Rectangle{0, 0, 0, 0}, map.zoom_level());

    // generate pose ranges to be checked
    auto pose_delta = RobotPoseDelta{0, 0, 0};
    auto translation_window = Rectangle{-max_y_error(), max_y_error(),
                                        -max_x_error(), max_x_error()};
    setup_init_zoom_level(map);
    for (double th = -max_th_error(); th <= max_th_error(); th += _ang_step) {
      pose_delta.theta = th;
      auto cost = sce->estimate_scan_cost(pose + pose_delta, scan, map);
      unchecked_poses.emplace(cost, th, translation_window, map.zoom_level());
    }

    return unchecked_poses;
  }

private:
  double _ang_step;
  double _tr_error_factor;
};

#endif // include guard
