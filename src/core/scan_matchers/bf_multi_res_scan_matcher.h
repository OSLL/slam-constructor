#ifndef SLAM_CTOR_CORE_BRUTE_FORCE_MULTIRES_SCAN_MATCHER_H
#define SLAM_CTOR_CORE_BRUTE_FORCE_MULTIRES_SCAN_MATCHER_H

#include <limits>
#include <queue>
#include <vector>
#include <utility>

#include "grid_scan_matcher.h"
#include "../geometry_primitives.h"

class BruteForceMultiResoultionScanMatcher : public GridScanMatcher {
private: // consts
  static constexpr double _Max_Translation_Error = 1,
                          _Max_Rotation_Error = deg2rad(5);
private: // types
  using SPEParams = ScanProbabilityEstimator::SPEParams;
  using Rect = decltype(SPEParams{}.sp_analysis_area);

  struct RobotPoseDeltas {
    // TODO: store info about init_pose, scan, map, approximator in the node
    double scan_prob_upper_bound;
    double rotation;
    Rect translations;

    RobotPoseDeltas(double sp, double th, const Rect& t_wn)
      : scan_prob_upper_bound{sp}, rotation{th}, translations{t_wn} {}

    // returns true if this node is _less_ prepefable than a given one
    bool operator<(const RobotPoseDeltas &other) const {
      if (!are_equal(scan_prob_upper_bound, other.scan_prob_upper_bound)) {
        // greater is "better" -> correctness
        return scan_prob_upper_bound < other.scan_prob_upper_bound;
      }
      if (!are_equal(translations.area(), other.translations.area())) {
        // finer is "better" -> speed up
        return translations.area() > other.translations.area();
      }
      // smaller is "better" -> fixes "blindness" of scan prob estimator
      return std::abs(rotation) > std::abs(other.rotation);
    }
  };

  std::ostream& print_pose(const RobotPoseDeltas &rbd) {
    return std::cout << "[" << deg2rad(rbd.rotation) << "]"
                     << "+ " << rbd.translations
                     << " -> " << rbd.scan_prob_upper_bound;
  }

  using UncheckedPoseDeltas = std::priority_queue<RobotPoseDeltas>;
public:
  BruteForceMultiResoultionScanMatcher(SPE est,
                                       double ang_step = deg2rad(0.1),
                                       double transl_step = 0.05)
    : GridScanMatcher{est, _Max_Translation_Error, _Max_Translation_Error,
                      _Max_Rotation_Error}
    , _ang_step{ang_step}, _transl_step{transl_step} {}

  void set_target_accuracy(double angle_step, double translation_step) {
    _ang_step = angle_step;
    _transl_step = translation_step;
  }

  double process_scan(const TransformedLaserScan &raw_scan,
                      const RobotPose &pose,
                      const GridMap &map,
                      RobotPoseDelta &result_pose_delta) override {
    double vanilla_scale = map.scale();
    auto scan = scan_probability_estimator()->filter_scan(raw_scan.scan, map);
    auto no_translation_prob = scan_probability(scan, pose, map);

    // FIXME: API - const cast to be able to do rescaling
    // NB: Do not make 'rescale' const (doing const_cast client probably
    //     won't forget to save/restore current scale)
    GridMap &rescalable_map = const_cast<GridMap&>(map);

    // TODO: dynamic angle step estimate
    add_scan_matching_request(pose, scan, rescalable_map, no_translation_prob);

    auto pose_deltas = find_best_pose_delta(pose, scan, rescalable_map,
                                            no_translation_prob);
    reset_scan_matching_requests();

    rescalable_map.rescale(vanilla_scale); // restore scale

    /* estimate best translation */
    auto best_translation = pose_deltas.translations.center();
    auto best_corr = RobotPoseDelta{best_translation.x, best_translation.y,
                                    pose_deltas.rotation};
    auto best_prob = scan_probability(scan, pose + best_corr, map);

    result_pose_delta = best_corr;
    return best_prob;
  }

private: // methods

  void add_scan_matching_request(const RobotPose &pose,
                                 const LaserScan2D &scan,
                                 GridMap &map, double threashold_sp) {
    auto rotation = RobotPoseDelta{0, 0, 0};
    // generate pose translation ranges to be checked
    const auto empty_trs_range = Rect{0, 0, 0, 0};
    const auto entire_trs_range = Rect{-max_y_error(), max_y_error(),
                                       -max_x_error(), max_x_error()};

    const double vanilla_scale = map.scale();
    for (double th = -max_th_error(); th <= max_th_error(); th += _ang_step) {
      rotation.theta = th;
      map.rescale(entire_trs_range.side());
      auto entire_sp = scan_probability(scan, pose + rotation, map,
                                        SPEParams{entire_trs_range});
      if (threashold_sp <= entire_sp) {
        _unchecked_pose_deltas.emplace(entire_sp, th, entire_trs_range);
      }

      // add explicit "no translation" entry
      map.rescale(vanilla_scale);
      auto fine_sp = scan_probability(scan, pose + rotation, map,
                                      SPEParams{empty_trs_range});
      if (threashold_sp <= entire_sp) {
        _unchecked_pose_deltas.emplace(fine_sp, th, empty_trs_range);
      }
    }
  }

  RobotPoseDeltas find_best_pose_delta(const RobotPose &pose,
                                       const LaserScan2D &scan,
                                       GridMap &map, double threashold_sp) {
    double vanilla_scale = map.scale();

    // the best pose lookup
    while (!_unchecked_pose_deltas.empty()) {
      auto d_poses = _unchecked_pose_deltas.top();
      auto should_branch_hor = _transl_step < d_poses.translations.hside_len();
      auto should_branch_vert = _transl_step < d_poses.translations.vside_len();
      if (!should_branch_vert && !should_branch_hor) {
        if (are_equal(d_poses.translations.area(), 0)) {
          return d_poses;
        } else {
          // replace the entry with actual points in the area
          // NB: center doesn't guarentee the optimal translation pick
          //     so (as a heuristic) test extra hypotheses in the rectangle.
          _unchecked_pose_deltas.pop();
          auto offsets = d_poses.translations.corners();
          offsets.push_back(d_poses.translations.center());

          map.rescale(vanilla_scale);
          for (const auto &offset : offsets) {
            auto corr = RobotPoseDelta{offset.x, offset.y, d_poses.rotation};
            auto prob = scan_probability(scan, pose + corr, map);
            auto range = Rect{offset.y, offset.y, offset.x, offset.x};
            if (prob <= threashold_sp) { continue; }
            // limit threashold with best-known-so-far probability
            threashold_sp = std::max(threashold_sp, prob);
            _unchecked_pose_deltas.emplace(prob, d_poses.rotation, range);
          }
        }
        continue;
      }

      // branching
      auto splitted_translations = std::vector<Rect>{d_poses.translations};
      if (should_branch_hor && should_branch_hor) {
        splitted_translations = d_poses.translations.split4_evenly();
      } else if (should_branch_hor) {
        splitted_translations = d_poses.translations.split_horiz();
      } else if (should_branch_vert) {
        splitted_translations = d_poses.translations.split_vert();
      }

      // update unchecked corrections
      _unchecked_pose_deltas.pop();
      for (auto& st : splitted_translations) {
        map.rescale(st.side()); // FIXME: non-squared areas handling
        Point2D best_translation = st.center();
        auto branch_delta = RobotPoseDelta{best_translation.x,
                                           best_translation.y,
                                           d_poses.rotation};
        auto branch_best_prob = scan_probability(scan, pose + branch_delta,
                                                 map, SPEParams{st});
        assert(branch_best_prob <= d_poses.scan_prob_upper_bound &&
               "BUG: Bounding assumption is violated");
        if (branch_best_prob <= threashold_sp) { continue; }
        _unchecked_pose_deltas.emplace(branch_best_prob, d_poses.rotation, st);
      }
    }
    assert(0 && "BUG: no pose delta has been found");
    return {std::numeric_limits<double>::quiet_NaN(), 0, Rect{}};
  }

  void reset_scan_matching_requests() {
    _unchecked_pose_deltas = UncheckedPoseDeltas{};
  }

private:
  UncheckedPoseDeltas _unchecked_pose_deltas;
  double _ang_step, _transl_step;
};

#endif // include guard
