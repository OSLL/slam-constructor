#ifndef SLAM_CTOR_CORE_POSE_ENUMERATORS_H
#define SLAM_CTOR_CORE_POSE_ENUMERATORS_H

#include <cmath>
#include "../states/robot_pose.h"

class PoseEnumerator {
public:
  virtual bool has_next() const = 0;
  virtual RobotPose next(const RobotPose &prev_pose) = 0;
  virtual void reset() {};
  virtual void feedback(bool /* pose_is_acceptable */) = 0;
  virtual ~PoseEnumerator() {}
};

class PolarCoordBruteForcePoseEnumerator : public PoseEnumerator {
public:
  PolarCoordBruteForcePoseEnumerator(
      double from_dir, double to_dir, double step_dir,
      double from_dst, double to_dst, double step_dst)
    : _base_pose_is_set{false}
    , _from_dir{from_dir}, _to_dir{to_dir}, _step_dir{step_dir}
    , _from_dst{from_dst}, _to_dst{to_dst}, _step_dst{step_dst} {
    assert(_from_dir <= _to_dir && _from_dst <= _to_dst);
    assert(0 < step_dir && 0 < step_dst);
    reset();
  }

  bool has_next() const override {
    return _dst <= _to_dst; // NB: "top-level" changing dimension
  }

  RobotPose next(const RobotPose &prev_pose) override {
    if (!_base_pose_is_set) {
      _base_pose = prev_pose;
      _base_pose_is_set = true;
    }

    auto delta = RobotPoseDelta{std::cos(_dir) * _dst,
                                std::sin(_dir) * _dst, 0};
    return _base_pose + delta;
  }

  void reset() override {
    _dir = _from_dir;
    _dst = _from_dst;
  }

  void feedback(bool pose_is_acceptable) override {
    // HACK: use switch falls to simplify code (no nested ifs)
    switch (0) {
    case 0:
      if (_dir < _to_dir) { _dir += _step_dir; break; }
      else                { _dir = _from_dir; /* to dst */ }
    case 1:
      _dst += _step_dst;
    }
  }

private:
  // TODO: use std::optional when C++17 is available
  bool _base_pose_is_set;
  RobotPose _base_pose;

  double _from_dir, _dir, _to_dir, _step_dir;
  double _from_dst, _dst, _to_dst, _step_dst;
};


#endif
