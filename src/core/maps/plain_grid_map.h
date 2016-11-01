#ifndef _PLAIN_GRID_MAP_H_INCLUDED
#define _PLAIN_GRID_MAP_H_INCLUDED

#include <cmath>
#include <vector>
#include <memory>
#include <cassert>

#include "grid_map.h"

class PlainGridMap : public GridMap {
public:
  // TODO: cp, mv ctors, dtor
  PlainGridMap(std::shared_ptr<GridCellFactory> cell_factory,
          int width = 1000, int height = 1000) :
    GridMap(cell_factory, width, height), _cells(height) {
    for (auto &row : _cells) {
      for (int i = 0; i < GridMap::width(); i++) {
        row.push_back(cell_factory->create_cell());
      }
    }
  }

  virtual void update_cell(
    const DiscretePoint2D& cell_coord,
    const GridCellValue &new_value, double quality = 1.0) {
    assert(has_cell(cell_coord));
    _cells[cell_coord.y][cell_coord.x]->set_value(new_value, quality);
  }

  virtual const GridCellValue &operator[](
    const DiscretePoint2D& cell_coord) const {
    assert(has_cell(cell_coord));
    return _cells[cell_coord.y][cell_coord.x]->value();
  }

  virtual bool has_cell(const DiscretePoint2D& cell_coord) const {
    return 0 <= cell_coord.x && cell_coord.y < width() &&
           0 <= cell_coord.y && cell_coord.y < height();
  }

private: // fields
  std::vector<std::vector<Cell>> _cells;
};

#endif
