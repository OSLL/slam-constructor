#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <memory>
#include <cmath>
#include <cassert>

#include "cell_occupancy_estimator.h"
#include "grid_cell_factory.h"
#include "../geometry_utils.h"

class GridMap {
public:
  using Cell = std::shared_ptr<GridCell>;
public:
  // TODO: cp, mv ctors, dtor
  GridMap(std::shared_ptr<GridCellFactory> cell_factory,
          int width, int height):
    // TODO: replace hardcoded value with params
    _width(width), _height(height), _m_per_cell(0.1),
    _cell_factory(cell_factory) {}

  int width() const { return _width; }
  int height() const { return _height; }
  double scale() const { return _m_per_cell; }
  double cell_scale() const { return _m_per_cell; }
  std::shared_ptr<GridCellFactory> cell_factory() const {
    return _cell_factory;
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

  virtual void update_cell(
    const DiscretePoint2D& cell_coord,
    const GridCellValue &new_value, double quality = 1.0) = 0;

  virtual const GridCellValue &operator[](
    const DiscretePoint2D& cell_coord) const = 0;

  virtual bool has_cell(const DiscretePoint2D& cell_coord) const = 0;

private: // fields
  int _width, _height;
  double _m_per_cell;
  std::shared_ptr<GridCellFactory> _cell_factory;
};

#endif
