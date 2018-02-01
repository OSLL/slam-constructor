#ifndef SLAM_CTOR_CORE_BRUTE_FORCE_SCAN_MATCHER_H
#define SLAM_CTOR_CORE_BRUTE_FORCE_SCAN_MATCHER_H

#include <cassert>

#include "pose_enumeration_scan_matcher.h"

// TODO: move to pose enumerators
// FIXME: class name
class BruteForcePoseEnumerator : public PoseEnumerator {
public:
  BruteForcePoseEnumerator(double from_x, double to_x, double step_x,
                           double from_y, double to_y, double step_y,
                           double from_t, double to_t, double step_t)
    : _base_pose_is_set{false}
    , _from_x{from_x}, _to_x{to_x}, _step_x{step_x}
    , _from_y{from_y}, _to_y{to_y}, _step_y{step_y}
    , _from_t{from_t}, _to_t{to_t}, _step_t{step_t} {
    assert(_from_x <= _to_x && _from_y <= _to_y && _from_t <= _to_t);
    reset();
  }

  bool has_next() const override {
    return _t <= _to_t; // NB: "top-level" changing dimension
  }

  RobotPose next(const RobotPose &prev_pose) override {
    if (!_base_pose_is_set) {
      _base_pose = prev_pose;
      _base_pose_is_set = true;
    }

    return {_base_pose.x + _x, _base_pose.y + _y, _base_pose.theta + _t};
  }

  void reset() override {
    _x = _from_x;
    _y = _from_y;
    _t = _from_t;
  }

  void feedback(bool pose_is_acceptable) override {
    // HACK: use switch falls to simplify code (no nested ifs/d_y, d_t tracking
    switch (0) {
    case 0:
      if (_x < _to_x) { _x += _step_x; break; }
      else            { _x = _from_x; /* to Y */ }
    case 1:
      if (_y < _to_y) { _y += _step_y; break; }
      else            { _y = _from_y; /* to T */ }
    case 2:
      _t += _step_t;
    }
  }

private:
  // TODO: use std::optional when C++17 is available
  bool _base_pose_is_set;
  RobotPose _base_pose;

  double _from_x, _x, _to_x, _step_x;
  double _from_y, _y, _to_y, _step_y;
  double _from_t, _t, _to_t, _step_t;
};

// TODO: add a PoseEnumerationScanMatcher descendant

class BruteForceScanMatcher : public PoseEnumerationScanMatcher {
public:
  BruteForceScanMatcher(std::shared_ptr<ScanProbabilityEstimator> estimator,
                        double from_x, double to_x, double step_x,
                        double from_y, double to_y, double step_y,
                        double from_t, double to_t, double step_t)
    : PoseEnumerationScanMatcher{
        estimator,
        std::make_shared<BruteForcePoseEnumerator>(from_x, to_x, step_x,
                                                   from_y, to_y, step_y,
                                                   from_t, to_t, step_t)
      } {}
};

#endif
