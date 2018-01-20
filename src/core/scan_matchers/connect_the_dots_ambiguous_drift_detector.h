#ifndef SLAM_CTOR_CORE_CONNECT_THE_DOTS_AMBIOUS_DRIFT_DETECTOR_H
#define SLAM_CTOR_CORE_CONNECT_THE_DOTS_AMBIOUS_DRIFT_DETECTOR_H

#include "grid_scan_matcher.h"
#include "pose_enumerators.h"
#include "../features/angle_histogram.h"

// TODO: do we need it to be a scan matcher?
// TEMP: implemented as a 'decorator' to be in sync with fmwk
class ConnectTheDotsAmbiguousDriftDetector : public GridScanMatcher {
public:
  ConnectTheDotsAmbiguousDriftDetector(std::shared_ptr<GridScanMatcher> sm)
    : GridScanMatcher{sm->scan_probability_estimator()}, _sm{sm} { }

  // GridScanMatcher API implementation

  void reset_state() override {
    _sm->reset_state();
  }

  double process_scan(const TransformedLaserScan &raw_scan,
                      const RobotPose &init_pose,
                      const GridMap &map,
                      RobotPoseDelta &pose_delta) override {
    // Estimate best pose delta with wrapped sm
    auto sm_best_prob = _sm->process_scan(raw_scan, init_pose, map, pose_delta);
    auto best_pose = init_pose + pose_delta;

    // Remove points with most popular antigradient
    auto scan = filter_scan(raw_scan.scan, init_pose, map);
    auto angle_histogram = AngleHistogram{20}.reset(scan);
    auto drift_dir_i = angle_histogram.max_i();
    auto drift_dir = angle_histogram[drift_dir_i];
    std::cout << drift_dir * 1.0 / scan.points().size() << std::endl;
    if (drift_dir < 0.4 * scan.points().size()) {
      return sm_best_prob;
    }
    std::cout << "ANALYSIS: " << drift_dir * 1.0 / scan.points().size() << std::endl;
    // TODO: max is close to avg => ok

    // TODO: move filtering to AngleHistogram
    LaserScan2D filtered_scan;
    filtered_scan.trig_provider = scan.trig_provider;
    auto &scan_pts = filtered_scan.points();
    for (LaserScan2D::Points::size_type i = 1; i < scan.points().size(); ++i) {
      auto &sp = scan.points()[i];
      // TODO: rm by ah index range
      if (drift_dir == angle_histogram.value(scan.points(), i)) {
        continue;
      }
      scan_pts.push_back(sp);
    }

    // Matching agaists filtered scan; BruteForce; move polar
    auto probs = std::vector<double>{};

    auto hist_step = angle_histogram.angle_step();
    // FIXME: a hack to exclude base_pose

    auto ang = drift_dir_i * hist_step + hist_step / 2;
    auto poses = PolarCoordBruteForcePoseEnumerator{
      //drift_dir_i * hist_step, (drift_dir_i + 1) * hist_step, hist_step / 2,
      ang, ang, hist_step,
      -0.25, 0.25, 0.1 /* Q: map scale? */
    };

    // TODO: Refactoring. Replace with BF sm + observer to log probs
    auto best_prob = scan_probability(filtered_scan, best_pose, map);
    while (poses.has_next()) {
      auto next_pose = poses.next(best_pose);
      auto next_prob = scan_probability(filtered_scan, next_pose, map);
      probs.push_back(next_prob);
      auto next_pose_is_better = best_prob < next_prob;
      if (next_pose_is_better) {
        best_prob = next_prob;
        best_pose = next_pose;
        std::cout << "2ND Pass Update: " << best_prob << " --> "
                  << best_pose << std::endl;
      }
      poses.feedback(next_pose_is_better);
    }

    if (1 < probs.size()) {
      std::sort(probs.begin(), probs.end(), std::greater<double>{});
      if (are_equal(probs[0], probs[1], 1e-6)) { // or both are shero
        std::cout << "ALARM: " << probs[0] << " == " << probs[1] << std::endl;
      }
    }

    pose_delta = best_pose - init_pose;
    auto corrected_best_prob = scan_probability(scan, best_pose, map);
    if (corrected_best_prob < sm_best_prob) {
      std::cout << "[CtD][Bug] " << corrected_best_prob << " "
                << sm_best_prob << std::endl;
    }
    return corrected_best_prob;
  }

private:
  std::shared_ptr<GridScanMatcher> _sm;
};


#endif
