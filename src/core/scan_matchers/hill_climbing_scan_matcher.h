#ifndef SLAM_CTOR_CORE_HILL_CLIMBING_SCAN_MATCHER_H
#define SLAM_CTOR_CORE_HILL_CLIMBING_SCAN_MATCHER_H

#include <memory>

#include "pose_enumeration_scan_matcher.h"
#include "../features/angle_histogram.h"

// TODO: move to pose enumerators
class Distorsion1DPoseEnumerator : public PoseEnumerator {
public:
  Distorsion1DPoseEnumerator(double translation_delta,
                             double rotation_delta,
                             double frame_rotation = 0)
    : _action_id{0}
    , _translation_delta{translation_delta}
    , _rotation_delta{rotation_delta}
    , _frame_rot_sin{std::sin(frame_rotation)}
    , _frame_rot_cos{std::cos(frame_rotation)}
    , _base_pose_is_set{false} {
    reset();
  }

  bool has_next() const override { return _action_id < DirNm * DimNm; }

  RobotPose next(const RobotPose &prev_pose) override {
    if (!_base_pose_is_set) {
      _base_pose = prev_pose;
      _base_pose_is_set = true;
    }
    auto distorted_pose = _base_pose;

    double dir = _action_id % DirNm ? -1 : 1;
    switch (_action_id % DimNm) {
    case X:
      distorted_pose.x +=  _frame_rot_cos * dir * _translation_delta;
      distorted_pose.y +=  _frame_rot_sin * dir * _translation_delta;
      break;
    case Y:
      distorted_pose.x += -_frame_rot_sin * dir * _translation_delta;
      distorted_pose.y +=  _frame_rot_cos * dir * _translation_delta;
      break;
    case Th:
      distorted_pose.theta += dir * _rotation_delta;
      break;
    default:
      assert(0 && "Missed shift destination");
    }

    ++_action_id;
    return distorted_pose;
  }

  void reset() override {}
  void feedback(bool pose_is_acceptable) override {}

private:
  enum Dim {X = 0, Y, Th, DimNm};
  enum Dir {Inc = 0, Dec, DirNm};
  static_assert(DimNm % 2 != DirNm % 2, "");
private:
  std::size_t _action_id;
  double _translation_delta, _rotation_delta;
  double _frame_rot_sin, _frame_rot_cos;

  RobotPose _base_pose;
  bool _base_pose_is_set;
};

template <typename BasePoseEnumerator>
class FailedRoundsLimitedPoseEnumerator : public PoseEnumerator {
public:
  FailedRoundsLimitedPoseEnumerator(unsigned max_failed_rounds,
                                    double translation_delta,
                                    double rotation_delta,
                                    double frame_rotation = 0)
    : _max_failed_rounds{max_failed_rounds}
    , _base_translation_delta{translation_delta}
    , _base_rotation_delta{rotation_delta} {
    reset();
  }

  bool has_next() const override {
    return _failed_rounds < _max_failed_rounds;
  }

  RobotPose next(const RobotPose &prev_pose) override {
    if (!_round_pe.has_next()) {
      if (_round_failed) {
        _translation_delta *= 0.5;
        _rotation_delta *= 0.5;
        ++_failed_rounds;
      }
      reset_round(_translation_delta, _rotation_delta);
    }
    return _round_pe.next(prev_pose);
  }

  void reset() override {
    _failed_rounds = 0;
    _translation_delta = _base_translation_delta;
    _rotation_delta = _base_rotation_delta;

    reset_round(_translation_delta, _rotation_delta);
  }

  void feedback(bool pose_is_acceptable) override {
    _round_failed &= !pose_is_acceptable;
    _round_pe.feedback(pose_is_acceptable);
  }

private:
  void reset_round(double translation_delta, double rotation_delta) {
    _round_pe = {_translation_delta, _rotation_delta};
    _round_failed = true;
  }
private:
  unsigned _max_failed_rounds;
  double _base_translation_delta, _base_rotation_delta;

  unsigned _failed_rounds;
  double _translation_delta, _rotation_delta;

  BasePoseEnumerator _round_pe{0, 0};
  bool _round_failed;
};

class HillClimbingScanMatcher : public PoseEnumerationScanMatcher {
private:
  using HCPE = FailedRoundsLimitedPoseEnumerator<Distorsion1DPoseEnumerator>;
public:
  // FIXME: update enumerator on set_lookup_ranges update
  HillClimbingScanMatcher(std::shared_ptr<ScanProbabilityEstimator> estimator,
                          unsigned max_lookup_failed_attempts,
                          double translation_delta, double rotation_delta,
                          double use_frame_alignement = false)
    : PoseEnumerationScanMatcher{
        estimator,
        std::make_shared<HCPE>(max_lookup_failed_attempts,
                               translation_delta, rotation_delta)}
    , _max_lookup_failed_attempts{max_lookup_failed_attempts}
    , _translation_delta{translation_delta}, _rotation_delta{rotation_delta}
    , _use_frame_alignement{use_frame_alignement} {}

  double process_scan(const TransformedLaserScan &raw_scan,
                      const RobotPose &init_pose,
                      const GridMap &map,
                      RobotPoseDelta &pose_delta) override {
    prepare_lookup(raw_scan.scan);
    return PoseEnumerationScanMatcher::process_scan(raw_scan, init_pose,
                                                    map, pose_delta);
  }
private:

  void prepare_lookup(const LaserScan2D &scan) {
    if (!_use_frame_alignement) { return; }
    auto hist = AngleHistogram{20};
    hist.reset(scan);
    double frame_rot = hist.major_direction();

    auto pe = std::make_shared<HCPE>(_max_lookup_failed_attempts,
        _translation_delta, _rotation_delta, frame_rot);
    set_pose_enumerator(pe);
  }

private:
  unsigned _max_lookup_failed_attempts;
  double _translation_delta, _rotation_delta;
  bool _use_frame_alignement;
};

#endif
