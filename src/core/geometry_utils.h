#ifndef __GEOMETRY_UTILS_H
#define __GEOMETRY_UTILS_H

#include <vector>
#include <cmath>

// Include *primitives.h in order to provide all geometry stuff with this header
#include "geometry_primitives.h"
#include "geometry_discrete_primitives.h"

class TrigonometricCache {
public:
 TrigonometricCache() :
    _sin_theta(0), _cos_theta(0),
    _angle_min(0), _angle_max(0), _angle_delta(0) {}

  inline double sin(double angle) const {
    int angle_idx = (angle - _angle_min) / _angle_delta;
    return _sin_theta * _cos[angle_idx] + _cos_theta * _sin[angle_idx];
  }

  inline double cos(double angle) const {
    int angle_idx = (angle - _angle_min) / _angle_delta;
    return _cos_theta * _cos[angle_idx] - _sin_theta * _sin[angle_idx];
  }

  void set_theta(double theta) {
    _sin_theta = std::sin(theta);
    _cos_theta = std::cos(theta);
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
  double _sin_theta, _cos_theta;
  double _angle_min, _angle_max, _angle_delta;
};

#endif
