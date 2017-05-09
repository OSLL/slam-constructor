#ifndef SLAM_CTOR_CORE_MANU_TO_MANY_MULTIRES_SCAN_MANCHR_H_INCLUDED
#define SLAM_CTOR_CORE_MANU_TO_MANY_MULTIRES_SCAN_MANCHR_H_INCLUDED

#include <limits>
#include <queue>
#include <vector>
#include <utility>

#include "grid_scan_matcher.h"
#include "../maps/grid_approximator.h"
#include "../geometry_primitives.h"

class Pow2BranchingPolicy {
public:

  static unsigned branchings_nm(double init_sz, double target_sz) {
    // FIXME: branchings estimate calculation
    // Motivation: performance (?)
    assert(target_sz <= init_sz);
    return unsigned(std::ceil(std::log(init_sz / target_sz)));
  }
};

class ManyToManyMultiResoultionScanMatcher : public GridScanMatcher {
private: // consts
  static constexpr double _Max_Translation_Error = 0.1,
                          _Max_Rotation_Error = deg2rad(2);
private: // types
  struct RobotPoseDeltas {
    // TODO: map ptr/ref
    double cost_lower_bound;
    double rotation;
    Rectangle translations;
    int zoom_lvl;

    RobotPoseDeltas(double c, double th, const Rectangle& t_wn, int l)
      : cost_lower_bound{c}, rotation{th}, translations{t_wn}, zoom_lvl{l} {}
    RobotPoseDeltas(const RobotPoseDeltas&) = default;
    RobotPoseDeltas(RobotPoseDeltas&&) = default;
    RobotPoseDeltas& operator=(const RobotPoseDeltas&) = default;
    RobotPoseDeltas& operator=(RobotPoseDeltas&&) = default;

    // returns true if this node is less prepefable than a given one
    bool operator<(const RobotPoseDeltas &other) const {
      if (cost_lower_bound == other.cost_lower_bound) {
        return zoom_lvl > other.zoom_lvl; // finer is "better"
      }
      // TODO: replace cost_lower_bound with probability in scan matcher
      return cost_lower_bound > other.cost_lower_bound; // smaller is "better"
    }
  };

  using UncheckedPoseDeltas = std::priority_queue<RobotPoseDeltas>;
  using MapApproximator = std::shared_ptr<OccupancyGridMapApproximator>;
public:
  // TODO: do we need max_*_error setup in ctor?
  ManyToManyMultiResoultionScanMatcher(std::shared_ptr<ScanCostEstimator> est,
                                       MapApproximator map_approximator,
                                       double ang_step = deg2rad(0.1),
                                       double tr_error_factor = 2)
    : GridScanMatcher{est, _Max_Translation_Error, _Max_Translation_Error,
                      _Max_Rotation_Error}
    , _ang_step{ang_step}, _tr_error_factor{tr_error_factor}
    , _map_approximator{map_approximator} {}

  double process_scan(const RobotPose &init_pose,
                      const TransformedLaserScan &scan,
                      const GridMap &map,
                      RobotPoseDelta &result_pose_delta) override {
    // TODO: dynamic angle step estimate
    add_scan_matching_request(init_pose, scan, map, _map_approximator);
    // TODO: store info about init_pose, scan, map, approximator in the node
    auto pose_deltas = find_best_pose_delta(init_pose, scan, map);
    reset_scan_matching_requests();

    auto translation = pose_deltas.translations.center();
    result_pose_delta = RobotPoseDelta{translation.x, translation.y,
                                       pose_deltas.rotation};
    return pose_deltas.cost_lower_bound;
  }

private: // methods
  unsigned estimate_sufficient_approximation_level(MapApproximator approx) {
    // FIXME
    /*
    auto t_error = max_translation_error(map);
    auto x_lvl = BranchingPolicy::branchings_nm(_translation_range_x, t_error);
    auto y_lvl = BranchingPolicy::branchings_nm(_translation_range_y, t_error);
    auto zoom_lvl = std::max(x_lvl, y_lvl);
    map.set_zoom_level(std::min(zoom_lvl, map.coarsest_zoom_level()));
    */
    return approx->max_approximation_level();
  }

  void add_scan_matching_request(const RobotPose &pose,
                                 const TransformedLaserScan &scan,
                                 const GridMap &map, MapApproximator approx) {
    // TODO: check approximator look for a given map
    auto sce = GridScanMatcher::cost_estimator();

    // add explicit "no correction" entry
    auto cost = sce->estimate_scan_cost(pose, scan, map);
    _unchecked_pose_deltas.emplace(cost, 0, Rectangle{0, 0, 0, 0}, 0);

    // generate pose ranges to be checked
    auto pose_delta = RobotPoseDelta{0, 0, 0};
    const auto translations = Rectangle{-max_y_error(), max_y_error(),
                                        -max_x_error(), max_x_error()};
    auto coarse_approx_lvl = estimate_sufficient_approximation_level(approx);
    auto &coarse_map = approx->map(coarse_approx_lvl);
    for (double th = -max_th_error(); th <= max_th_error(); th += _ang_step) {
      pose_delta.theta = th;
      auto cost = sce->estimate_scan_cost(pose + pose_delta, scan, coarse_map);
      _unchecked_pose_deltas.emplace(cost, th, translations, coarse_approx_lvl);
    }
  }

  double max_translation_error(const GridMap &map) {
    return map.scale() / _tr_error_factor;
  }

  RobotPoseDeltas find_best_pose_delta(const RobotPose &pose,
                                       const TransformedLaserScan &scan,
                                       const GridMap &map) {
    auto sce = GridScanMatcher::cost_estimator();
    auto acceptable_transl_error = max_translation_error(map);
    // the best pose lookup
    while (!_unchecked_pose_deltas.empty()) {
      auto d_poses = _unchecked_pose_deltas.top();
      if (d_poses.zoom_lvl == 0) {
         // search is done
        return d_poses;
      }

      // branching
      auto& tr_wn = d_poses.translations;
      auto splitted_translations = std::vector<Rectangle>{};
      auto should_branch = acceptable_transl_error < tr_wn.vside_len() ||
                           acceptable_transl_error < tr_wn.hside_len();
      if (should_branch) {
        splitted_translations = tr_wn.split4_evenly();
      } else {
        splitted_translations.push_back(tr_wn);
      }

      // update unchecked corrections
      _unchecked_pose_deltas.pop();
      auto &coarse_map = _map_approximator->map(d_poses.zoom_lvl - 1);
      for (auto& st : splitted_translations) {
        // TODO: Robustness guarantee the translation is the best in the window
        Point2D best_translation = st.center();
        auto branch_delta = RobotPoseDelta{best_translation.x,
                                           best_translation.y,
                                           d_poses.rotation};
        auto branch_cost = sce->estimate_scan_cost(pose + branch_delta,
                                                   scan, coarse_map);
        assert(d_poses.cost_lower_bound <= branch_cost &&
               "BUG: Bounding assumption is violated");
        _unchecked_pose_deltas.emplace(branch_cost, d_poses.rotation, st,
                                       d_poses.zoom_lvl - 1);
      }
    }
    // std::cout << "M3RS End ?!" << std::endl;
    // BUG: no pose delta has been found
    return {std::numeric_limits<double>::quiet_NaN(), 0, Rectangle{}, -1};
  }

  void reset_scan_matching_requests() {
    _unchecked_pose_deltas = UncheckedPoseDeltas{};
  }

private:
  UncheckedPoseDeltas _unchecked_pose_deltas;
  double _ang_step;
  double _tr_error_factor;
  MapApproximator _map_approximator;
};

#endif // include guard
