#ifndef __GEOMETRY_PRIMITIVES_H_INCLUDED
#define __GEOMETRY_PRIMITIVES_H_INCLUDED

#include <cmath>
#include <cassert>
#include <algorithm>
#include <ostream>

#include "math_utils.h"

struct Point2D {
  Point2D(double x_par = 0, double y_par = 0) : x(x_par), y(y_par) {}

  double dist_sq(const Point2D &pt) const {
    return std::pow(x - pt.x, 2) + std::pow(y - pt.y, 2);
  }

  bool operator==(const Point2D &that) const {
    return are_equal(x, that.x) && are_equal(y, that.y);
  }

  Point2D operator*(double scalar) const { return {scalar * x, scalar * y}; }
  Point2D operator+(const Point2D& p) const { return {x + p.x, y + p.y}; }
  Point2D operator-(const Point2D& p) const { return {x - p.x, y - p.y}; }

public: // fields
  double x, y;
};

Point2D operator*(double scalar, const Point2D &p) { return p * scalar; }

std::ostream &operator<<(std::ostream &stream, const Point2D &pnt) {
  return stream << "(" << pnt.x << ", " << pnt.y << ")";
}

class Segment2D {
public: //methods

  static Segment2D invalid() {
    Segment2D invalid = {{0, 0}, {0, 0}};
    invalid.is_valid = false;
    return invalid;
  }

  Segment2D(const Point2D &begin, const Point2D &end) : _beg{begin}, _end{end} {
    _is_horiz = are_equal(_beg.y, _end.y);
    _is_vert = are_equal(_beg.x, _end.x);
  }

  bool is_horiz() const { return _is_horiz; }
  bool is_vert() const { return _is_vert; }
  bool is_point() const { return _is_horiz && _is_vert; }

  const Point2D& beg() const { return _beg; }
  const Point2D& end() const { return _end; }

  explicit operator bool() const { return is_valid; }

  bool contains(const Point2D &p) const {
    if (is_horiz()) {
      return are_equal(p.y, _beg.y) && are_ordered(_beg.x, p.x, _end.x);
    }
    if (is_vert()) {
      return are_equal(p.x, _beg.x) && are_ordered(_beg.y, p.y, _end.y);
    }
    assert(false && "BUG: Contains is undefined for non-axes-aligned segment");
    return false;
  }

  bool contains_intersection(const Point2D &p) const {
    return x_projection_contains(p) && y_projection_contains(p);
  }

  double length_sq() const {
    return std::pow(_end.x - _beg.x, 2) + std::pow(_end.y - _beg.y, 2);
  }

private: // methods

  bool x_projection_contains(const Point2D &p) const {
    return are_ordered(_beg.x, p.x, _end.x) || are_ordered(_end.x, p.x, _beg.x);
  }

  bool y_projection_contains(const Point2D &p) const {
    return are_ordered(_beg.y, p.y, _end.y) || are_ordered(_end.y, p.y, _beg.y);
  }

private: // fields
  bool is_valid = true;
  Point2D _beg, _end;
  bool _is_horiz, _is_vert;
};

std::ostream &operator<<(std::ostream &stream, const Segment2D &s) {
  return stream << "[" << s.beg() << "; " << s.end() << "]";
}

struct Intersection : public Point2D {
  enum class Location : char {
    Bot = 0, Left = 1, Top = 2, Right = 3
  };

  Intersection(Location loc, double x, double y) :
    Point2D{x, y}, location(loc) {}

  bool is_horiz() const {
    return location == Location::Bot || location == Location::Top;
  }

  Location location;
};

using Intersections = std::vector<Intersection>;

struct Ray { // in parametric form
  Ray(double x_s, double x_d, double y_s, double y_d) :
    beg{x_s, y_s}, delta{x_d, y_d} {}

  explicit Ray(const Segment2D &s) : beg(s.beg()), delta(s.end() - s.beg()) {}

public:
  void intersect(const Segment2D &s, Intersection::Location loc,
                 Intersections &consumer) const {
    if (s.is_horiz()) {
      intersect_horiz_segm(s.beg().x, s.end().x, s.beg().y, loc, consumer);
      return;
    }
    if (s.is_vert()) {
      intersect_vert_segm(s.beg().y, s.end().y, s.beg().x, loc, consumer);
      return;
    }
    assert(false && "BUG: Unable to intersect non-axes-aligned segment");
  }

private:
  void intersect_horiz_segm(double st_x, double end_x, double y,
                            Intersection::Location loc,
                            Intersections &consumer) const {
    if (are_equal(delta.y, 0)) { return; }

    double inters_alpha = (y - beg.y) / delta.y;
    double inters_x = beg.x + inters_alpha * delta.x;
    if (inters_x < st_x || end_x < inters_x) // out of segment bounds
      return;

    consumer.emplace_back(loc, inters_x, y);
  }

  void intersect_vert_segm(double st_y, double end_y, double x,
                           Intersection::Location loc,
                           Intersections &consumer) const {
    if (are_equal(delta.x, 0)) { return; }

    double inters_alpha = (x - beg.x) / delta.x;
    double inters_y = beg.y + inters_alpha * delta.y;
    if (inters_y < st_y || end_y < inters_y) // out of segment bounds
      return;

    consumer.emplace_back(loc, x, inters_y);
  }
private: // fields
  Point2D beg, delta;
};

struct Rectangle {
  Rectangle() : Rectangle(0, 0, 0, 0) {}
  Rectangle(double b, double t, double l, double r) :
    bot{b}, top{t}, left{l}, right{r},
    bot_edge{{left, bot}, {right, bot}}, top_edge{{left, top}, {right, top}},
    left_edge{{left, bot}, {left, top}}, right_edge{{right, bot}, {right, top}}
  {
    assert(bot <= top);
    assert(left <= right);
  }

  double side() const { return top - bot; }
  double area() const { return (top - bot) * (right - left); }

  /* Inclusion predicates */
  bool contains(const Point2D &p) const {
    return are_ordered(bot, p.y, top) && are_ordered(left, p.x, right);
  }

  // NB: segment is not necessary coincide with an edge of the rectangle
  bool has_on_edge_line(const Segment2D &s) const {
    if (s.is_vert()) {
      return are_equal(s.beg().x, left) || are_equal(s.beg().x, right);
    }
    if (s.is_horiz()) {
      return are_equal(s.beg().y, bot) || are_equal(s.beg().y, top);
    }
    return false;
  }

  Segment2D find_containing_edge(const Point2D &p) const {
    for (auto &e : edges()) {
      if (!e.contains(p)) { continue; }
      return e;
    }
    return Segment2D::invalid();
  }

  /* Intersections lookup */

  Intersections find_intersections(const Segment2D &s) const {
    Intersections ray_intrs = find_intersections(Ray{s});
    Intersections inters;
    std::copy_if(ray_intrs.begin(), ray_intrs.end(), std::back_inserter(inters),
      [&s](const Intersection &i){ return s.contains_intersection(i); });
    return inters;
  }

  Intersections find_intersections(const Ray &ray) const {
    Intersections intersections;

    // NB: order matters for correct vertex intersection handling
    ray.intersect(top_edge, Intersection::Location::Top, intersections);
    ray.intersect(left_edge, Intersection::Location::Left, intersections);
    ray.intersect(bot_edge, Intersection::Location::Bot, intersections);
    ray.intersect(right_edge, Intersection::Location::Right, intersections);

    // filter dublicate intersection (intersection is a vertex)
    if (1 < intersections.size() &&
        intersections.front() == intersections.back()) {
      intersections.pop_back();
    }
    auto new_last = std::unique(intersections.begin(), intersections.end());
    intersections.erase(new_last, intersections.end());
    assert(intersections.size() < 3);
    return intersections;
  }

// TODO: make fields private
public: // fields
  const double bot, top, left, right;

private: // methods
  std::vector<Segment2D> edges() const {
    return {top_edge, bot_edge, left_edge, right_edge};
  }
private: // fields
  // NB: edges as Segment2D were introduced for code clarity.
  //     They can be dropped for performance reasons
  // TODO: vector of edges
  Segment2D bot_edge, top_edge, left_edge, right_edge;
};

std::ostream &operator<<(std::ostream &stream, const Rectangle &r) {
  stream << "Rectangle [t:" << r.top << ", b:" << r.bot;
  return stream << ", l:" << r.left << ", r:" << r.right << "]";
}

#endif
