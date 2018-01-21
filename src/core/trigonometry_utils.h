#ifndef SLAM_CTOR_CORE_TRIGONOMETRY_UTILS_H
#define SLAM_CTOR_CORE_TRIGONOMETRY_UTILS_H

#include <vector>
#include <cmath>

// NB: polymorphic-provider soluction is cleaner from the OOP PoV.
//     in case of performance issues related to 'virtual calls'
//     can be replaced with 'ifs' in clients.
class TrigonometryProvider {
public:
  virtual double sin(double angle_rad) const = 0;
  virtual double cos(double angle_rad) const = 0;
  virtual void set_base_angle(double angle_rad) = 0;
};

class RawTrigonometryProvider : public TrigonometryProvider {
public:
  RawTrigonometryProvider() : _base_angle{0} {}

  double sin(double angle_rad) const override {
    return std::sin(_base_angle + angle_rad);
  }

  double cos(double angle_rad) const override {
    return std::cos(_base_angle + angle_rad);
  }

  void set_base_angle(double angle_rad) override {
    _base_angle = angle_rad;
  }

private:
  double _base_angle;
};

class CachedTrigonometryProvider : public TrigonometryProvider {
public:
  CachedTrigonometryProvider()
    : _sin_base(0), _cos_base(0)
    , _angle_min(0), _angle_max(0), _angle_delta(0) {
    set_base_angle(0); // 'virtual call' is not virtual here, but it's ok.
  }

  double sin(double angle_rad) const override {
    // std::round is crucial to deal with inaccurate fp values
    int angle_idx = std::round((angle_rad - _angle_min) / _angle_delta);
    return _sin_base * _cos[angle_idx] + _cos_base * _sin[angle_idx];
  }

  double cos(double angle_rad) const override {
    // std::round is crucial to deal with inaccurate fp values
    int angle_idx = std::round((angle_rad - _angle_min) / _angle_delta);
    return _cos_base * _cos[angle_idx] - _sin_base * _sin[angle_idx];
  }

  void set_base_angle(double angle_rad) override {
    _sin_base = std::sin(angle_rad);
    _cos_base = std::cos(angle_rad);
  }

  void update(double a_min, double a_max, double a_inc) {
    if (a_min == _angle_min && a_max == _angle_max && a_inc == _angle_delta)
      return;
    _sin.clear();
    _cos.clear();
    _angle_min = a_min;
    _angle_max = a_max;
    _angle_delta = a_inc;

    int angles_nm = (_angle_max - _angle_min) / _angle_delta + 1;
    _sin.reserve(angles_nm);
    _cos.reserve(angles_nm);
    for(double angle = _angle_min; angle < _angle_max; angle += _angle_delta) {
      _sin.push_back(std::sin(angle));
      _cos.push_back(std::cos(angle));
    }
  }

private:
  std::vector<double> _sin, _cos;
  double _sin_base, _cos_base;
  double _angle_min, _angle_max, _angle_delta;
};

#endif
