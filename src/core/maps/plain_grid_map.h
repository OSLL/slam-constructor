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
  PlainGridMap(std::shared_ptr<GridCell> prototype,
          int width = 1000, int height = 1000) :
    GridMap(prototype, width, height), _cells(height) {
    for (auto &row : _cells) {
      row.reserve(GridMap::width());
      for (int i = 0; i < GridMap::width(); i++) {
        row.push_back(prototype->clone());
      }
    }
  }

  virtual GridCell &operator[] (const DPnt2D& c) override {
    assert(has_cell(c));
    DPnt2D coord = outer2internal(c);
    return *_cells[coord.y][coord.x];
  }

  virtual const GridCell &operator[](const DPnt2D& c) const override {
    assert(has_cell(c));
    DPnt2D coord = outer2internal(c);
    return *_cells[coord.y][coord.x];
  }

private: // fields
  std::vector<std::vector<std::unique_ptr<GridCell>>> _cells;
};

#endif
