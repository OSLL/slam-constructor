#ifndef SLAM_CTOR_CORE_REGULAR_SQUARES_GRID_H
#define SLAM_CTOR_CORE_REGULAR_SQUARES_GRID_H

#include <cmath>
#include <cassert>
#include <utility>
#include <vector>
#include <limits>

#include "../math_utils.h"
#include "../geometry_utils.h"

class RegularSquaresGrid {
public:
  using Coord = DiscretePoint2D;
  static constexpr double Dbl_Inf = std::numeric_limits<double>::infinity();
public:

  RegularSquaresGrid(int w, int h, double scale) :
    _width{w}, _height{h}, _m_per_cell(scale) {}

  RegularSquaresGrid(const RegularSquaresGrid &grid) = default;
  RegularSquaresGrid& operator=(const RegularSquaresGrid &grid) = default;
  RegularSquaresGrid(RegularSquaresGrid &&grid) = default;
  RegularSquaresGrid& operator=(RegularSquaresGrid &&grid) = default;
  virtual ~RegularSquaresGrid() = default;

  // TODO: change return type to unsigned
  virtual int width() const { return _width; }
  virtual int height() const { return _height; }
  virtual double scale() const { return _m_per_cell; }
  virtual void rescale(double /*cell_size*/) {}

  Coord world_to_cell(const Point2D &pt) const {
    return world_to_cell(pt.x, pt.y);
  }

  // FIXME: code duplication for perfromance reasons.
  //        Find out the reason why world_to_cell(x, y, scale) runs slower
  Coord world_to_cell(double x, double y) const {
    // TODO: handle infinity
    assert(x != Dbl_Inf &&  y != Dbl_Inf);

    auto curr_scale = scale();
    return {int(std::floor(x / curr_scale)), int(std::floor(y / curr_scale))};
  }

  Coord world_to_cell(double x, double y, double scale) const {
    // TODO: handle infinity
    assert(x != Dbl_Inf &&  y != Dbl_Inf);

    return {int(std::floor(x / scale)), int(std::floor(y / scale))};
  }

  // TODO: move resterization to a separate component
  std::vector<Coord> world_to_cells(const Segment2D &s) const {
    // returns a vector of cells intersected by a given segment.
    // The first cell contains start of the segment, the last - its end.
    // algorithm: modified 4-connection line
    double d_x = s.end().x - s.beg().x, d_y = s.end().y - s.beg().y;
    int inc_x = 0 < d_x ? 1 : -1, inc_y = 0 < d_y ? 1 : -1;

    Coord pnt = world_to_cell(s.beg());
    const Coord end = world_to_cell(s.end());
    size_t cells_nm = std::abs(end.x - pnt.x) + std::abs(end.y - pnt.y) + 1;
    std::vector<Coord> cells;
    cells.reserve(cells_nm);

    double m_per_cell = scale();
    Point2D mid_cell{(pnt.x + 0.5) * m_per_cell, (pnt.y + 0.5) * m_per_cell};
    // NB: y's are multiplied by d_x to eliminate separate vert. line handling
    auto mid_cell_seg_y = d_x * s.beg().y + (mid_cell.x - s.beg().x) * d_y;
    auto e = mid_cell_seg_y - mid_cell.y * d_x; // e = actual_e * d_x
    auto e_x_inc = inc_x * m_per_cell * d_y; // slope * d_x
    auto e_y_inc = -inc_y * m_per_cell * d_x; // - d_x

    while (1) {
      cells.push_back(pnt);
      if (pnt == end) { break; }
      if (cells_nm < cells.size()) { // fail-over (fp rounding errors)
        return DiscreteSegment2D{world_to_cell(s.beg()), end};
      }

      double e_x = e + e_x_inc, e_y = e + e_y_inc;
      double abs_err_diff = std::abs(e_y) - std::abs(e_x);
      if (are_equal(abs_err_diff, 0)) {
        // through cell diagonal or coord grid's line aligned
        if      (pnt.x == end.x) { pnt.y += inc_y; }
        else if (pnt.y == end.y) { pnt.x += inc_x; }
        else { pnt.x += inc_x; pnt.y += inc_y; }
        e = 0;
      } else if (0 < abs_err_diff) {
        pnt.x += inc_x;
        e = e_x;
      } else { // abs_err_diff < 0
        pnt.y += inc_y;
        e = e_y;
      }
    }
    return cells;
  }

  Point2D cell_to_world(const Coord &cell) const {
    return {scale() * (cell.x + 0.5), scale() * (cell.y + 0.5)};
  }

  // TODO: consider renaming ~"occupied_space"
  Rectangle world_cell_bounds(const Coord &coord) const {
    auto m_per_cell = scale();
    if (m_per_cell == Dbl_Inf) {
      return {-Dbl_Inf, Dbl_Inf, -Dbl_Inf, Dbl_Inf};
    }

    return {m_per_cell * coord.y,        // bot
            m_per_cell * (coord.y + 1),  // top
            m_per_cell * coord.x,        // left
            m_per_cell * (coord.x + 1)}; // right
  }

  virtual DiscretePoint2D origin() const {
    return DiscretePoint2D{(int)width() / 2, (int)height() / 2};
  }

  virtual bool has_cell(const Coord &c) const {
    return has_internal_cell(external2internal(c));
  }

  Coord internal2external(const Coord &coord) const {
    return coord - origin();
  }

protected:

  bool has_internal_cell(const Coord &c) const {
    return (0 <= c.x && c.x < width()) and (0 <= c.y && c.y < height());
  }

  // A cell coordinate determined outside of the map to the coord on the grid
  // Motivation: grid's structure changes after world_to_cell coord return
  //             can spoil the returned coord.
  Coord external2internal(const Coord &coord) const {
    return coord + origin();
  }

  // notifies with actual height/width changes
  void set_height(unsigned h) { _height = h; }
  void set_width(unsigned w) { _width = w; }
  void set_scale(double s) { _m_per_cell = s; }

private: // fields
  int _width, _height;
  double _m_per_cell;
};

#endif
