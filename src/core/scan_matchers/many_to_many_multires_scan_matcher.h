#ifndef SLAM_CTOR_CORE_MANY_TO_MANY_MULTIRES_SCAN_MANCHR_H_INCLUDED
#define SLAM_CTOR_CORE_MANY_TO_MANY_MULTIRES_SCAN_MANCHR_H_INCLUDED

#include <limits>
#include <queue>
#include <vector>
#include <utility>

#include "grid_scan_matcher.h"
#include "../maps/grid_approximator.h"
#include "../geometry_primitives.h"

#include "../../utils/console_view.h"

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
  static constexpr double _Max_Translation_Error = 1,
                          _Max_Rotation_Error = deg2rad(5);
private: // types
  struct RobotPoseDeltas {
    // TODO: map ptr/ref
    double scan_prob_upper_bound;
    double rotation;
    Rectangle translations;
    int zoom_lvl;

    RobotPoseDeltas(double sp, double th, const Rectangle& t_wn, int l)
      : scan_prob_upper_bound{sp}, rotation{th}, translations{t_wn}
      , zoom_lvl{l} {}

    // returns true if this node is _less_ prepefable than a given one
    bool operator<(const RobotPoseDeltas &other) const {
      if (are_equal(scan_prob_upper_bound, other.scan_prob_upper_bound)) {
        return zoom_lvl > other.zoom_lvl; // finer is "better"
      }
      // greater is "better"
      return scan_prob_upper_bound < other.scan_prob_upper_bound;
    }
  };

  using UncheckedPoseDeltas = std::priority_queue<RobotPoseDeltas>;
  using MapApproximator = std::shared_ptr<OccupancyGridMapApproximator>;
  using SPEParams = ScanProbabilityEstimator::SPEParams;
public:
  // TODO: do we need max_*_error setup in ctor?
  ManyToManyMultiResoultionScanMatcher(SPE est,
                                       MapApproximator map_approximator,
                                       double ang_step = deg2rad(0.1),
                                       double tr_error_factor = 2)
    : GridScanMatcher{est, _Max_Translation_Error, _Max_Translation_Error,
                      _Max_Rotation_Error}
    , _ang_step{ang_step}, _tr_error_factor{tr_error_factor}
    , _map_approximator{map_approximator} {}

  double process_scan(const TransformedLaserScan &scan,
                      const RobotPose &init_pose,
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
    return pose_deltas.scan_prob_upper_bound;
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

    // add explicit "no correction" entry
    // TODO: add base prob. for different angles
    auto scan_prob = scan_probability(scan, pose, map);
    _unchecked_pose_deltas.emplace(scan_prob, 0, Rectangle{0, 0, 0, 0}, 0);

    // generate pose ranges to be checked
    auto pose_delta = RobotPoseDelta{0, 0, 0};
    const auto translations = Rectangle{-max_y_error(), max_y_error(),
                                        -max_x_error(), max_x_error()};
    auto coarse_approx_lvl = estimate_sufficient_approximation_level(approx);
    auto &coarse_map = approx->map(coarse_approx_lvl);
    for (double th = -max_th_error(); th <= max_th_error(); th += _ang_step) {
      pose_delta.theta = th;
      auto sp = scan_probability(scan, pose + pose_delta, coarse_map,
                                 SPEParams{translations});
      //std::cout << th << " => " << sp << std::endl;
      _unchecked_pose_deltas.emplace(sp, th, translations, coarse_approx_lvl);
    }
  }

  double max_translation_error(const GridMap &map) {
    return map.scale() / _tr_error_factor;
  }

  RobotPoseDeltas find_best_pose_delta(const RobotPose &pose,
                                       const TransformedLaserScan &scan,
                                       const GridMap &map) {
    auto acceptable_transl_error = max_translation_error(map);
    // the best pose lookup
    while (!_unchecked_pose_deltas.empty()) {
      auto d_poses = _unchecked_pose_deltas.top();
      //std::cout << d_poses.zoom_lvl << ": " << d_poses.rotation
      //          << " -> " << d_poses.scan_prob_upper_bound << std::endl;
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
      //auto &coarse_map = _map_approximator->map(d_poses.zoom_lvl - 1);
      //std::cout << "Scale: " << coarse_map.scale() << std::endl;
      //show_grid_map(coarse_map, Point2D{pose.x, pose.y}, 5, 5);
      for (auto& st : splitted_translations) {
        // TODO: Robustness guarantee the translation is the best in the window
        Point2D best_translation = st.center();
        auto branch_delta = RobotPoseDelta{best_translation.x,
                                           best_translation.y,
                                           d_poses.rotation};
        auto branch_best_prob = scan_probability(scan, pose + branch_delta,
                                                 map, SPEParams{st});
        //std::cout << "  ++ Add (" << d_poses.zoom_lvl - 1 << "): "
        //          << branch_best_prob << " in " << st << std::endl;
        assert(branch_best_prob <= d_poses.scan_prob_upper_bound &&
               "BUG: Bounding assumption is violated");
        _unchecked_pose_deltas.emplace(branch_best_prob, d_poses.rotation, st,
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
