#ifndef SLAM_CTOR_CORE_GEOMETRY_DISCRETE_PRIMITIVES_H
#define SLAM_CTOR_CORE_GEOMETRY_DISCRETE_PRIMITIVES_H

#include <cmath>
#include <vector>
#include <utility>
#include <ostream>
#include <tuple>

struct DiscretePoint2D {
public: // fields
  int x, y;
public: // functions
  constexpr DiscretePoint2D(int x_coord = 0, int y_coord = 0)
    : x{x_coord}, y{y_coord} {}

  DiscretePoint2D &operator+=(const DiscretePoint2D &p) {
    x += p.x;
    y += p.y;
    return *this;
  }

  DiscretePoint2D operator+(const DiscretePoint2D &p) const {
    return DiscretePoint2D{*this} += p;
  }

  DiscretePoint2D operator-(const DiscretePoint2D &p) const {
    return {x - p.x, y - p.y};
  }

  bool operator==(const DiscretePoint2D &p) const {
    return x == p.x && y == p.y;
  }

  bool operator!=(const DiscretePoint2D &p) const {
    return !(*this == p);
  }

  DiscretePoint2D operator-() const {
    return {-x, -y};
  }

  double dist_sq(const DiscretePoint2D &pt) const {
    return std::pow(x - pt.x, 2) + std::pow(y - pt.y, 2);
  }
};

inline std::ostream &operator<<(std::ostream &stream,
                                const DiscretePoint2D &pnt) {
  return stream << "(" << pnt.x << ", " << pnt.y << ")";
}

//------------------

class DiscreteSegment2D {
  using DPoint = DiscretePoint2D;
public: // methods
  DiscreteSegment2D(const DPoint &beg, const DPoint &end) {
    // pricise pts nm: max(abs(d_x), abs(d_y)) + 1
    _points.reserve(std::fabs(beg.x - end.x) + std::fabs(beg.y - end.y) + 1);
    gen_points_with_bresenham(beg, end);
  }
  operator auto() const { return _points; } // to vector of discrete points

private: // methods

  void gen_points_with_bresenham(const DPoint &beg, const DPoint &end) {
    // 1. Setup vars according to line direction (d_x, d_y)
    DPoint delta = end - beg;
    bool y_is_primary = std::abs(delta.x) < std::abs(delta.y);

    int limit, primary, d_primary, secondary, d_secondary;
    std::tie(limit, primary, d_primary, secondary, d_secondary) = y_is_primary ?
      std::make_tuple(end.y, beg.y, delta.y, beg.x, delta.x) :
      std::make_tuple(end.x, beg.x, delta.x, beg.y, delta.y);

    int inc_primary = 0 < d_primary ? 1 : -1;
    int inc_secondary = 0 < d_secondary ? 1 : -1;
    int *x = nullptr, *y = nullptr;
    std::tie(x, y) = y_is_primary ? std::make_tuple(&secondary, &primary) :
                                    std::make_tuple(&primary, &secondary);

    // 2. Generate points. Driver: pnt = err_min(next_pnt, next_diag_pnt)
    int error = 0; // e = actual e * d_primary
    while (1) {
      _points.emplace_back(*x, *y);
      if (primary == limit) { break; }

      int err_inc_primary = error + inc_primary * d_secondary; // e' = e + slope
      int err_inc_both = err_inc_primary - inc_secondary * d_primary; // e' - 1

      primary += inc_primary;
      if (std::abs(err_inc_primary) < std::abs(err_inc_both)) {
        error = err_inc_primary;
      } else {
        secondary += inc_secondary;
        error = err_inc_both;
      }
    }
  }

private: // fields
  std::vector<DPoint> _points;
};

#endif
