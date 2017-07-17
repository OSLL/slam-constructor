#ifndef SLAM_CTOR_CORE_GEOMETRY_PRIMITIVES_H
#define SLAM_CTOR_CORE_GEOMETRY_PRIMITIVES_H

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

inline Point2D operator*(double scalar, const Point2D &p) { return p * scalar; }

inline std::ostream &operator<<(std::ostream &stream, const Point2D &pnt) {
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

inline std::ostream &operator<<(std::ostream &stream, const Segment2D &s) {
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

struct LightWeightRectangle {
private:
  using LVRect = LightWeightRectangle;
public:
  LightWeightRectangle() : LightWeightRectangle(0, 0, 0, 0) {}
  LightWeightRectangle(double b, double t, double l, double r)
    : _bot{b}, _top{t}, _left{l}, _right{r} {
    assert(_bot <= _top);
    assert(_left <= _right);
  }
  LightWeightRectangle(const Point2D &p)
    : LightWeightRectangle(p.y, p.y, p.x, p.x) {}

  double bot() const { return _bot; }
  double top() const { return _top; }
  double left() const { return _left; }
  double right() const { return _right; }

  double vside_len() const { return top() - bot(); }
  double hside_len() const { return right() - left(); }
  bool is_square() const { return vside_len() == hside_len(); }
  double side() const { return vside_len(); }
  double area() const { return vside_len() * hside_len(); }
  Point2D center() const {
    return {left() + hside_len() / 2, bot() + vside_len() / 2};
  }

  bool is_line() const { return (vside_len() == 0) ^ (hside_len() == 0); }
  bool is_point() const { return (vside_len() == 0) && (hside_len() == 0); }

  bool operator==(const LightWeightRectangle &rhs) const {
    return are_equal(top(), rhs.top()) && are_equal(bot(), rhs.bot()) &&
           are_equal(left(), rhs.left()) && are_equal(right(), rhs.right());
  }

  auto corners() const {
    return std::vector<Point2D>{{left(), bot()}, {left(), top()},
                                {right(), bot()}, {right(), top()}};
  }

  LVRect move_center(const Point2D &new_center) const {
    double half_v = vside_len() / 2, half_h = hside_len() / 2;
    return {new_center.y - half_v, new_center.y + half_v,
            new_center.x - half_h, new_center.x + half_h};
  }

  auto shrink(double factor) const {
    auto c = center();
    auto new_hv = vside_len() / (factor * 2),
         new_hh = hside_len() / (factor * 2);
    return LVRect{c.y - new_hv, c.y + new_hv,
                  c.x - new_hh, c.x + new_hh};

  }

  auto split_vert() const {
    auto c = center();
    return std::vector<LVRect>{
      LVRect{bot(),   c.y, left(), right()},
      LVRect{  c.y, top(), left(), right()}
    };
  }

  auto split_horz() const {
    auto c = center();
    return std::vector<LVRect>{
      LVRect{bot(), top(), left(),     c.x},
      LVRect{bot(), top(),    c.x, right()}
    };
  }

  auto split4_evenly() const {
    auto c = center();
    return std::vector<LVRect>{
      LVRect{ bot(),   c.y, left(),     c.x}, // left-bot
      LVRect{   c.y, top(), left(),     c.x}, // left-top
      LVRect{ bot(),   c.y,    c.x, right()}, // right-bot
      LVRect{   c.y, top(),    c.x, right()}  // right-top
    };
  }

  bool contains(const Point2D &p) const { return contains(p.x, p.y); }

  bool contains(double x, double y) const {
    return are_ordered(left(), x, right()) && are_ordered(bot(), y, top());
  }

  auto intersect(const LightWeightRectangle &that) const {
    return intersect_internal(that, false);
  }

  auto overlap(const LightWeightRectangle &that) const {
    if (area()) {
      return intersect(that).area() / area();
    }
    if (that.area()) {
      return that.contains(left(), bot()) ? 1.0 : 0.0;
    }
    // FIXME: the assert below, see unit tests.
    assert(is_point() && that.is_point() && "TODO: support lwr-lines");
    return *this == that ? 1.0 : 0.0;
  }

private:

  LightWeightRectangle intersect_internal(const LightWeightRectangle &that,
                                          bool reversed = false) const {
    // NB: a naive implementation; straightforward and correct (I hope so).
    // IDEA: shrink -this- to intersection rectangle.

    // FIXME: @see unit tests
    unsigned contained_corners_nm = 0;
    auto cleft = left(), cright = right(), ctop = top(), cbot = bot();

    #define PROCESS_CORNER(horz_id, vert_id)                    \
      if (contains(that.horz_id(), that.vert_id())) {           \
        ++contained_corners_nm;                                 \
        c##horz_id = that.horz_id();                            \
        c##vert_id = that.vert_id();                            \
      }

    PROCESS_CORNER(left, bot);
    PROCESS_CORNER(right, bot);
    PROCESS_CORNER(left, top);
    PROCESS_CORNER(right, top);

    #undef PROCESS_CORNER

    // handle corner cases based on a number of contained corners (pun intended)
    switch (contained_corners_nm) {
    case 0:
      /*
       * Either doesn't overlap or the overlap must be detected
       * by this-in-that corners analysis (reversed).
       */
      // FIXME: awkward stuff
      return reversed ? LightWeightRectangle{}
                      : that.intersect_internal(*this, true);
    case 1: case 2: case 4:
      break;
    case 3: default:
      assert(0 && "Unexpected inclusions number");
    }

    return LightWeightRectangle{cbot, ctop, cleft, cright};
  }

private: // fields
  double _bot, _top, _left, _right;
};

inline std::ostream &operator<<(std::ostream &stream,
                                const LightWeightRectangle &r) {
  stream << "LVRectangle [t:" << r.top() << ", b:" << r.bot();
  return stream << ", l:" << r.left() << ", r:" << r.right() << "]";
}

struct Rectangle : LightWeightRectangle {
private: // consts
  static constexpr std::size_t BOT_EDGE_IDX = 0;
  static constexpr std::size_t TOP_EDGE_IDX = 1;
  static constexpr std::size_t LFT_EDGE_IDX = 2;
  static constexpr std::size_t RHT_EDGE_IDX = 3;
public: // methods

  Rectangle() : Rectangle(0, 0, 0, 0) {}
  Rectangle(double b, double t, double l, double r)
    : LightWeightRectangle{b, t, l, r}
    , _edges{
        Segment2D{{ left(), bot()}, {right(), bot()}}, // 0 - bot
        Segment2D{{ left(), top()}, {right(), top()}}, // 1 - top
        Segment2D{{ left(), bot()}, { left(), top()}}, // 2 - left
        Segment2D{{right(), bot()}, {right(), top()}}  // 3 - right
      } {}

  Rectangle(const Rectangle &) = default;
  Rectangle(Rectangle &&) = default;
  Rectangle& operator=(const Rectangle &rhs) = default;
  Rectangle& operator=(Rectangle &&rhs) = default;

  /* Inclusion predicates */

  // NB: segment is not necessary coincide with an edge of the rectangle
  bool has_on_edge_line(const Segment2D &s) const {
    if (s.is_vert()) {
      return are_equal(s.beg().x, left()) || are_equal(s.beg().x, right());
    }
    if (s.is_horiz()) {
      return are_equal(s.beg().y, bot()) || are_equal(s.beg().y, top());
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
    ray.intersect(top_edge(), Intersection::Location::Top, intersections);
    ray.intersect(left_edge(), Intersection::Location::Left, intersections);
    ray.intersect(bot_edge(), Intersection::Location::Bot, intersections);
    ray.intersect(right_edge(), Intersection::Location::Right, intersections);

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

private: // methods
  const Segment2D& bot_edge()   const { return edges()[BOT_EDGE_IDX]; }
  const Segment2D& top_edge()   const { return edges()[TOP_EDGE_IDX]; }
  const Segment2D& left_edge()  const { return edges()[LFT_EDGE_IDX]; }
  const Segment2D& right_edge() const { return edges()[RHT_EDGE_IDX]; }

  const std::vector<Segment2D> &edges() const { return _edges; }
private: // fields
  // NB: edges as Segment2D were introduced for code clarity.
  //     They can be dropped for performance reasons
  const std::vector<Segment2D> _edges;
};

#endif
