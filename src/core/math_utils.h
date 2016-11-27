#ifndef __SLAM_CTOR_CORE_MATH_UTILS_H_INCLUDED
#define __SLAM_CTOR_CORE_MATH_UTILS_H_INCLUDED

inline bool are_equal(double a, double b) {
  // cmp doubles according to http://realtimecollisiondetection.net/blog/?p=89
  double eps_scale = std::max(1.0, std::max(std::abs(a), std::abs(b)));
  constexpr double Eps = 1e-7; // num_limits::epsilon is too small
  return std::abs(a - b) <= Eps * eps_scale;
}

inline bool less_or_equal(double a, double b) {
  return are_equal(a, b) || (a < b);
}

inline bool are_ordered(double a, double b, double c) {
  return less_or_equal(a, b) && less_or_equal(b, c);
}

#endif
