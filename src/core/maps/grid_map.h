#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <memory>
#include <cmath>

#include "cell_occupancy_estimator.h"
#include "grid_cell_factory.h"
#include "../geometry_utils.h"

class GridMap {
public:
  using Cell = std::shared_ptr<GridCell>;
public:
  // TODO: cp, mv ctors, dtor
  GridMap(std::shared_ptr<GridCellFactory> cell_factory,
          int width = 1000, int height = 1000):
    // TODO: replace hardcoded value with params
    _width(width), _height(height), _m_per_cell(0.1),
    _cell_factory(cell_factory), _cells(_height) {
    for (auto &row : _cells) {
      for (int i = 0; i < _width; i++) {
        row.push_back(cell_factory->create_cell());
      }
    }
  }

  int width() const { return _width; }
  int height() const { return _height; }
  double scale() const { return _m_per_cell; }

  const std::vector<std::vector<Cell>> cells() const { return _cells; }

  void update_cell(const DiscretePoint2D& cell_coord,
                   const GridCellValue &new_value, double quality = 1.0) {
    assert(has_cell(cell_coord));
    _cells[cell_coord.y][cell_coord.x]->set_value(new_value, quality);
  }

  const GridCellValue &operator[](const DiscretePoint2D& cell_coord) const {
    return cell_value(cell_coord);
  }

  const GridCellValue &cell_value(const DiscretePoint2D& cell_coord) const {
    assert(has_cell(cell_coord));
    return _cells[cell_coord.y][cell_coord.x]->value();
  }

  bool has_cell(const DiscretePoint2D& cell_coord) const {
    return 0 <= cell_coord.x && cell_coord.y < _width &&
           0 <= cell_coord.y && cell_coord.y < _height;
  }

  DiscretePoint2D world_to_cell(double x, double y) const {
    int cell_x = std::floor(_width /2 + x/_m_per_cell);
    int cell_y = std::floor(_height/2 + y/_m_per_cell);

    return DiscretePoint2D(cell_x, cell_y);
  }

  double cell_scale() const { return _m_per_cell; }

  Rectangle world_cell_bounds(const DiscretePoint2D &cell_coord) {
    assert(has_cell(cell_coord));
    Rectangle bounds;
    bounds.bot = (cell_coord.y + _height/2) * _m_per_cell;
    bounds.top = bounds.bot + _m_per_cell;
    bounds.left = (cell_coord.x + _width/2) * _m_per_cell;
    bounds.right = bounds.left + _m_per_cell;
    return bounds;
  }

private: // fields
  int _width, _height;
  double _m_per_cell;
  std::shared_ptr<GridCellFactory> _cell_factory;
  std::vector<std::vector<Cell>> _cells;
};

#endif
