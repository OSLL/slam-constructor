#ifndef __GEOMETRY_UTILS_H
#define __GEOMETRY_UTILS_H

#include <vector>
#include <cmath>

#define EQ_DOUBLE(a, b)                         \
  (std::abs((a) - (b)) < 0.0001)

inline double QUAD(double x) {
  return x*x;
}
inline double normal_variation(double dx, double dy, double dz = 0,
                                                             double sig = 1) {
    return std::exp(-0.5 * (QUAD(dx) + QUAD(dy) + QUAD(dz))/QUAD(sig));
  }

struct Rectangle {
  Rectangle() : Rectangle(0, 0, 0, 0) {}
  Rectangle(double b, double t, double l, double r) :
    bot(b), top(t), left(l), right(r) {}

  bool does_contain(double x, double y) const {
    return ((bot < y) && (y < top)) && ((left < x) && (x < right));
  }

  double area() const {
    return (top - bot)*(right - left);
  }

  double bot, top, left, right;
};

struct Point2D {
  Point2D(double x_par = 0, double y_par = 0) : x(x_par), y(y_par) {}
  double dist_sq(const Point2D &pt) const {
    return std::pow(x - pt.x, 2) + std::pow(y - pt.y, 2);
  }
  double x, y;
};

struct Beam {
  Point2D beg, end;
};

struct DiscretePoint2D {
public:
  DiscretePoint2D(int x_coord = 0, int y_coord = 0):
    x{x_coord}, y{y_coord} {}
  // TODO: mv (!!), cpy ctors
  int x, y;

  DiscretePoint2D operator+(const DiscretePoint2D &p) const {
    return DiscretePoint2D(x + p.x, y + p.y);
  }

  DiscretePoint2D operator-(const DiscretePoint2D &p) const {
    return DiscretePoint2D(x - p.x, y - p.y);
  }

  bool operator==(const DiscretePoint2D &p) const {
    return x == p.x && y == p.y;
  }

  DiscretePoint2D operator-() const {
    return DiscretePoint2D(-x, -y);
  }

  double dist_sq(const DiscretePoint2D &pt) const {
    return std::pow(x - pt.x, 2) + std::pow(y - pt.y, 2);
  }
};

class DiscreteLine2D {
  using Point = DiscretePoint2D;
public: // methods
  DiscreteLine2D(const Point &start, const Point &end) {
    _points.reserve(std::fabs(start.x - end.x + 1) +
                    std::fabs(start.y - end.y + 1));
    generatePointsWithBresenham(start.x, start.y, end.x, end.y);
  }
  std::vector<Point>& points() { return _points; }
private:
  void generatePointsWithBresenham(int x1, int y1, int x2, int y2) {
    // TODO: copypasted from
    //   http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm
    //   review and simplification are required

    int delta_x(x2 - x1);
    // if x1 == x2, then it does not matter what we set here
    signed char const ix((delta_x > 0) - (delta_x < 0));
    delta_x = std::abs(delta_x) * 2;

    int delta_y(y2 - y1);
    // if y1 == y2, then it does not matter what we set here
    signed char const iy((delta_y > 0) - (delta_y < 0));
    delta_y = std::abs(delta_y) * 2;

    _points.push_back(Point(x1, y1));

    if (delta_x >= delta_y) {
      // error may go below zero
      int error(delta_y - (delta_x >> 1));
      while (x1 != x2) {
        if ((0 <= error) && (error || (0 < ix))) {
          error -= delta_x;
          y1 += iy;
        }
        // else do nothing
        error += delta_y;
        x1 += ix;
        _points.push_back(Point(x1, y1));
      }
    }
    else {
      // error may go below zero
      int error(delta_x - (delta_y >> 1));

      while (y1 != y2) {
        if ((0 <= error) && (error || (0 < iy))) {
          error -= delta_y;
          x1 += ix;
        }
        // else do nothing
        error += delta_x;
        y1 += iy;
        _points.push_back(Point(x1, y1));
      }
    }
  }
private: // fields
  std::vector<Point> _points;
};

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
