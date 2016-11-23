#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <memory>
#include <cmath>
#include <cassert>

#include "cell_occupancy_estimator.h"
#include "grid_cell.h"
#include "../geometry_utils.h"

struct GridMapParams {
  int width, height; // width & height in meters
  double meters_per_cell;
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
  // TODO: cp, mv ctors, dtor
  GridMap(std::shared_ptr<GridCell> prototype,
          const GridMapParams& params = MapValues::gmp) :
    _width(params.width), _height(params.height),
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
  double cell_scale() const { return _m_per_cell; }

  std::unique_ptr<GridCell> new_cell() const {
    return _cell_prototype->clone();
  }

  DiscretePoint2D world_to_cell(const Point2D &pt) const {
    return world_to_cell(pt.x, pt.y);
  }

  DiscretePoint2D world_to_cell(double x, double y) const {
    int cell_x = std::floor(x/_m_per_cell);
    int cell_y = std::floor(y/_m_per_cell);
    return abs2internal({cell_x, cell_y});
  }

  Rectangle world_cell_bounds(const DiscretePoint2D &c) const {
    assert(has_cell(c));

    DiscretePoint2D cell_coord = outer2internal(c);
    Rectangle bounds;
    bounds.bot = cell_coord.y * _m_per_cell;
    bounds.top = bounds.bot + _m_per_cell;
    bounds.left = cell_coord.x * _m_per_cell;
    bounds.right = bounds.left + _m_per_cell;
    return bounds;
  }

  // Absolute cell coord (world frame) to cell coord on the grid
  virtual DiscretePoint2D abs2internal(const DiscretePoint2D &coord) const {
    return coord + origin();
  }

  virtual DiscretePoint2D origin() const {
    static DiscretePoint2D origin{(int)_width / 2, (int)_height / 2};
    return origin;
  }

  virtual bool has_cell(const DiscretePoint2D& c) const {
    DiscretePoint2D cell_coord = outer2internal(c);
    return 0 <= cell_coord.x && cell_coord.x < width() &&
           0 <= cell_coord.y && cell_coord.y < height();

  }

  virtual GridCell &operator[](const DPnt2D& coord) = 0;
  virtual const GridCell &operator[](const DPnt2D& coord) const = 0;

protected:

  // A cell coordinate determined outside of the map to the coord on the grid
  // Motivation: grid's structure changes after a abs2internal coord return
  //             can spoil the returned coord.
  virtual DiscretePoint2D outer2internal(const DiscretePoint2D &coord) const {
    return coord;
  }

  void set_height(unsigned h) { _height = h; }
  void set_width(unsigned w) { _width = w; }
private: // fields
  int _width, _height;
  double _m_per_cell;
  std::shared_ptr<GridCell> _cell_prototype;
};

#endif
