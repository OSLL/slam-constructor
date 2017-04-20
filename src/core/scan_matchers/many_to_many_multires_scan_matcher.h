#ifndef SLAM_CTOR_CORE_MANU_TO_MANY_MULTIRES_SCAN_MANCHR_H_INCLUDED
#define SLAM_CTOR_CORE_MANU_TO_MANY_MULTIRES_SCAN_MANCHR_H_INCLUDED

#include <limits>
#include <queue>
#include <vector>
#include <utility>

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
public:
  // TODO: do we need max_*_error setup in ctor?
  ManyToManyMultiResoultionScanMatcher(std::shared_ptr<ScanCostEstimator> est,
                                       double ang_step = deg2rad(0.1),
                                       double tr_error_factor = 2)
    : GridScanMatcher{est, _Max_Translation_Error, _Max_Translation_Error,
                      _Max_Rotation_Error}
    , _ang_step{ang_step}, _tr_error_factor{tr_error_factor} {}

  double process_scan(const RobotPose &init_pose,
                      const TransformedLaserScan &scan,
                      const GridMap &map,
                      RobotPoseDelta &result_pose_delta) override {
    // TODO: dynamic angle step estimate
    //std::cout << "M3RS Start" << std::endl;
    // FIXME?: dynamic cast; iterators over zoomed maps?
    auto &z_map = dynamic_cast<ZoomableGridMap<CoreMapType> &>(
                    const_cast<GridMap&>(map));

    add_scan_matching_request(init_pose, scan, z_map);
    // TODO: store info about init_pose, scan, z_map in the node
    auto pose_deltas = find_best_pose_delta(init_pose, scan, z_map);
    reset_scan_matching_requests();

    auto translation = pose_deltas.translations.center();
    result_pose_delta = RobotPoseDelta{translation.x, translation.y,
                                       pose_deltas.rotation};
    return pose_deltas.cost_lower_bound;
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

  void add_scan_matching_request(const RobotPose &pose,
                                 const TransformedLaserScan &scan,
                                 ZoomableGridMap<CoreMapType> &map) {
    auto sce = GridScanMatcher::cost_estimator();
    // add explicit "no correction" entry
    map.set_zoom_level(map.finest_zoom_level());
    auto cost = sce->estimate_scan_cost(pose, scan, map);
    _unchecked_pose_deltas.emplace(cost, 0, Rectangle{0, 0, 0, 0},
                                   map.zoom_level());

    // generate pose ranges to be checked
    auto pose_delta = RobotPoseDelta{0, 0, 0};
    const auto translations = Rectangle{-max_y_error(), max_y_error(),
                                        -max_x_error(), max_x_error()};
    setup_init_zoom_level(map);
    for (double th = -max_th_error(); th <= max_th_error(); th += _ang_step) {
      pose_delta.theta = th;
      auto cost = sce->estimate_scan_cost(pose + pose_delta, scan, map);
      _unchecked_pose_deltas.emplace(cost, th, translations, map.zoom_level());
    }
  }

  double max_translation_error(ZoomableGridMap<CoreMapType> &map) {
    map.set_zoom_level(map.finest_zoom_level());
    return map.scale() / _tr_error_factor;
  }

  RobotPoseDeltas find_best_pose_delta(const RobotPose &pose,
                                       const TransformedLaserScan &scan,
                                       ZoomableGridMap<CoreMapType> &map) {
    auto sce = GridScanMatcher::cost_estimator();
    auto acceptable_transl_error = max_translation_error(map);
    // the best pose lookup
    while (!_unchecked_pose_deltas.empty()) {
      auto d_poses = _unchecked_pose_deltas.top();
      if (d_poses.zoom_lvl == ZoomableGridMap<GridMap>::finest_zoom_level()) {
         // search is done
        return d_poses;
      }

      // branching
      auto& tr_wn = d_poses.translations;
      auto splitted_translations = std::vector<Rectangle>{};
      auto should_branch = acceptable_transl_error < tr_wn.vside_len() ||
                           acceptable_transl_error < tr_wn.hside_len();
      if (should_branch) {
        splitted_translations = BranchingPolicy::branch(tr_wn);
      } else {
        splitted_translations.push_back(tr_wn);
      }

      // update unchecked corrections
      _unchecked_pose_deltas.pop();
      map.set_zoom_level(d_poses.zoom_lvl - 1);
      for (auto& st : splitted_translations) {
        // TODO: Robustness guarantee the translation is the best in the window
        Point2D best_translation = st.center();
        auto branch_delta = RobotPoseDelta{best_translation.x,
                                           best_translation.y,
                                           d_poses.rotation};
        auto branch_cost = sce->estimate_scan_cost(pose + branch_delta,
                                                   scan, map);
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
};

#endif // include guard
