#ifndef SLAM_CTOR_CORE_MATH_UTILS_H
#define SLAM_CTOR_CORE_MATH_UTILS_H

#include <cmath>
#include <limits>

// Workaround to enable compiling on gcc-4.9
#if __GNUC__ < 5
#define CONSTEXPR
#else
#define CONSTEXPR constexpr
#endif

// TODO: add unit tests for math utils

template <typename T>
CONSTEXPR inline bool are_equal(const T& a, const T& b, const T& eps) {
  return std::abs(a - b) <= eps;
}

// FIXME: are_equal(infinity, 0) is true
CONSTEXPR inline bool are_equal(double a, double b) {
  // cmp doubles according to http://realtimecollisiondetection.net/blog/?p=89
  double eps_scale = std::max(1.0, std::max(std::abs(a), std::abs(b)));
  constexpr double Eps = 1e-7; // num_limits::epsilon is too small
  return are_equal(a, b, Eps * eps_scale);
}

CONSTEXPR inline bool less(double a, double b) {
  constexpr double Eps = std::numeric_limits<double>::epsilon();
  return a < b + Eps;
}

CONSTEXPR inline bool is_multiple_of(double value, double factor) {
  // TODO: verify numerical safety/bounds
  double ratio = value / factor;
  return are_equal(ratio, std::trunc(ratio));
}

inline double bound_value(double left, double v, double right) {
  return std::max(left, std::min(right, v));
}

inline bool less_or_equal(double a, double b) {
  return are_equal(a, b) || less(a, b);
}

inline bool are_strictly_ordered(double a, double b, double c) {
  return less(a, b) && less(b, c);
}

inline bool are_ordered(double a, double b, double c) {
  return less_or_equal(a, b) && less_or_equal(b, c);
}

constexpr inline double deg2rad(double angle_deg) {
  return angle_deg * M_PI / 180;
}

constexpr inline double rad2deg(double angle_rad) {
  return angle_rad * 180 / M_PI;
}

template <unsigned N>
CONSTEXPR int ge_pow(int i) {
  int ge_p = 1;
  while (ge_p < i) {
    ge_p *= N;
  }
  return ge_p;
}

#endif
