#ifndef SLAM_CTOR_CORE_GRID_UTILS_H_INCLUDED
#define SLAM_CTOR_CORE_GRID_UTILS_H_INCLUDED

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

  Coord world_to_cell(const Point2D &pt) const {
    return world_to_cell(pt.x, pt.y);
  }

  Coord world_to_cell(double x, double y) const {
    int cell_x = std::floor(x / scale());
    int cell_y = std::floor(y / scale());
    return {cell_x, cell_y};
  }

  Point2D cell_to_world(const Coord &cell) const {
    return {scale() * (cell.x + 0.5), scale() * (cell.y + 0.5)};
  }

  Coord world_to_cell_by_vec(double v_origin_x, double v_origin_y,
                             double range, double direction_a) const {
    double x_world = v_origin_x + range * std::cos(direction_a);
    double y_world = v_origin_y + range * std::sin(direction_a);
    return world_to_cell(x_world, y_world);
  }

  std::vector<Coord> coords_in_area(const Rectangle &area) const {
    auto left_bot = world_to_cell(area.left(), area.bot());
    auto right_top = world_to_cell(area.right(), area.top());

    std::vector<Coord> coords;
    coords.reserve((right_top.x - left_bot.x) * (right_top.y - left_bot.y));
    for (int x = left_bot.x; x < right_top.x; ++x) {
      for (int y = left_bot.y; y < right_top.y; ++y) {
        coords.emplace_back(x, y);
      } // y
    } // x
    return coords;
  }

  std::vector<Coord> valid_coords() const {
    auto left_bot = cell_to_world(internal2external({0, 0}));
    auto right_top = cell_to_world(internal2external({width(), height()}));
    return coords_in_area({left_bot.y, right_top.y,
                           left_bot.x, right_top.x});
  }

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

  Rectangle world_cell_bounds(const Coord &coord) const {
    assert(has_cell(coord));

    auto m_per_cell = scale();
    assert(m_per_cell != std::numeric_limits<double>::infinity());
    return {m_per_cell * coord.y,        // bot
            m_per_cell * (coord.y + 1),  // top
            m_per_cell * coord.x,        // left
            m_per_cell * (coord.x + 1)}; // right
  }

  virtual DiscretePoint2D origin() const {
    return DiscretePoint2D{(int)width() / 2, (int)height() / 2};
  }

  virtual bool has_cell(const Coord &c) const {
    DiscretePoint2D coord = external2internal(c);
    return 0 <= coord.x && coord.x < width() &&
           0 <= coord.y && coord.y < height();

  }

  Coord internal2external(const Coord &coord) const {
    return coord - origin();
  }

protected:
  // A cell coordinate determined outside of the map to the coord on the grid
  // Motivation: grid's structure changes after world_to_cell coord return
  //             can spoil the returned coord.
  Coord external2internal(const Coord &coord) const {
    return coord + origin();
  }

  // notifies with actual height/width changes
  void set_height(unsigned h) { _height = h; }
  void set_width(unsigned w) { _width = w; }

private: // fields
  int _width, _height;
  double _m_per_cell;
};

#endif
