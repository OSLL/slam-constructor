#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <memory>
#include <cmath>
#include <cassert>
#include <vector>
#include <utility>

#include "cell_occupancy_estimator.h"
#include "grid_cell.h"
#include "../geometry_utils.h"

struct GridMapParams {
  const int width_cells, height_cells;
  const double meters_per_cell;
};

struct MapValues {
  static constexpr int width = 1000;
  static constexpr int height = 1000;
  static constexpr double meters_per_cell = 0.1;
  static constexpr GridMapParams gmp{width, height, meters_per_cell};
};

class GridMap {
public:
  using DPnt2D = DiscretePoint2D;
public:
  GridMap(std::shared_ptr<GridCell> prototype,
          const GridMapParams& params = MapValues::gmp) :
    _width(params.width_cells), _height(params.height_cells),
    _m_per_cell(params.meters_per_cell), _cell_prototype(prototype) {}

  GridMap(GridMap &gm) = default;
  GridMap& operator=(GridMap &gm) = default;
  GridMap(GridMap &&gm) = default;
  GridMap& operator=(GridMap &&gm) = default;
  virtual ~GridMap() = default;

  // TODO: change return type to unsigned
  int width() const { return _width; }
  int height() const { return _height; }
  double scale() const { return _m_per_cell; }

  std::unique_ptr<GridCell> new_cell() const {
    return _cell_prototype->clone();
  }

  DiscretePoint2D world_to_cell(const Point2D &pt) const {
    return world_to_cell(pt.x, pt.y);
  }

  DiscretePoint2D world_to_cell(double x, double y) const {
    int cell_x = std::floor(x/_m_per_cell);
    int cell_y = std::floor(y/_m_per_cell);
    return {cell_x, cell_y};
  }

  std::vector<DiscretePoint2D> world_to_cells(const Segment2D &s) const {
    // returns a vector of cells intersected by a given segment.
    // The first cell contains start of the segment, the last - its end.
    // algorithm: modified 4-connection line
    double d_x = s.end().x - s.beg().x, d_y = s.end().y - s.beg().y;
    int inc_x = 0 < d_x ? 1 : -1, inc_y = 0 < d_y ? 1 : -1;

    DiscretePoint2D pnt = world_to_cell(s.beg());
    const DiscretePoint2D end = world_to_cell(s.end());
    size_t cells_nm = std::abs(end.x - pnt.x) + std::abs(end.y - pnt.y) + 1;
    std::vector<DiscretePoint2D> cells;
    cells.reserve(cells_nm);

    Point2D mid_cell{(pnt.x + 0.5)* _m_per_cell, (pnt.y + 0.5) * _m_per_cell};
    // NB: y's are multiplied by d_x to eliminate separate vert. line handling
    auto mid_cell_seg_y = d_x * s.beg().y + (mid_cell.x - s.beg().x) * d_y;
    auto e = mid_cell_seg_y - mid_cell.y * d_x; // e = actual_e * d_x
    auto e_x_inc = inc_x * _m_per_cell * d_y; // slope * d_x
    auto e_y_inc = -inc_y * _m_per_cell * d_x; // - d_x

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

  Rectangle world_cell_bounds(const DiscretePoint2D &cell_coord) const {
    assert(has_cell(cell_coord));
    return {_m_per_cell * cell_coord.y,        // bot
            _m_per_cell * (cell_coord.y + 1),  // top
            _m_per_cell * cell_coord.x,        // left
            _m_per_cell * (cell_coord.x + 1)}; // right
  }

  virtual DiscretePoint2D origin() const {
    static DiscretePoint2D origin{(int)_width / 2, (int)_height / 2};
    return origin;
  }

  virtual bool has_cell(const DiscretePoint2D& c) const {
    DiscretePoint2D cell_coord = external2internal(c);
    return 0 <= cell_coord.x && cell_coord.x < width() &&
           0 <= cell_coord.y && cell_coord.y < height();

  }

  virtual GridCell &operator[](const DPnt2D& coord) = 0;
  virtual const GridCell &operator[](const DPnt2D& coord) const = 0;

protected:

  // A cell coordinate determined outside of the map to the coord on the grid
  // Motivation: grid's structure changes after world_to_cell coord return
  //             can spoil the returned coord.
  DPnt2D external2internal(const DPnt2D &p) const { return p + origin(); }

  void set_height(unsigned h) { _height = h; }
  void set_width(unsigned w) { _width = w; }
private: // fields
  int _width, _height;
  double _m_per_cell;
  std::shared_ptr<GridCell> _cell_prototype;
};

#endif
