#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <memory>
#include <cmath>
#include <cassert>

#include "cell_occupancy_estimator.h"
#include "grid_cell.h"
#include "../geometry_utils.h"

class GridMap {
public:
  using DPnt2D = DiscretePoint2D;
public:
  // TODO: cp, mv ctors, dtor
  GridMap(std::shared_ptr<GridCell> prototype,
          int width, int height):
    // TODO: replace hardcoded value with params
    _width(width), _height(height), _m_per_cell(0.1),
    _cell_prototype(prototype) {}

  GridMap(GridMap &gm) = default;
  GridMap& operator=(GridMap &gm) = default;
  GridMap(GridMap &&gm) = default;
  GridMap& operator=(GridMap &&gm) = default;
  virtual ~GridMap() = default;

  int width() const { return _width; }
  int height() const { return _height; }
  double scale() const { return _m_per_cell; }
  double cell_scale() const { return _m_per_cell; }

  std::unique_ptr<GridCell> new_cell() const {
    return _cell_prototype->clone();
  }

  DiscretePoint2D world_to_cell(double x, double y) const {
    int cell_x = std::floor(_width /2 + x/_m_per_cell);
    int cell_y = std::floor(_height/2 + y/_m_per_cell);

    return DiscretePoint2D(cell_x, cell_y);
  }

  Rectangle world_cell_bounds(const DiscretePoint2D &cell_coord) {
    assert(has_cell(cell_coord));
    Rectangle bounds;
    bounds.bot = (cell_coord.y + _height/2) * _m_per_cell;
    bounds.top = bounds.bot + _m_per_cell;
    bounds.left = (cell_coord.x + _width/2) * _m_per_cell;
    bounds.right = bounds.left + _m_per_cell;
    return bounds;
  }

  virtual GridCell &operator[](const DPnt2D& coord) = 0;
  virtual const GridCell &operator[](const DPnt2D& coord) const = 0;
  virtual bool has_cell(const DiscretePoint2D& cell_coord) const = 0;

private: // fields
  int _width, _height;
  double _m_per_cell;
  std::shared_ptr<GridCell> _cell_prototype;
};

#endif
