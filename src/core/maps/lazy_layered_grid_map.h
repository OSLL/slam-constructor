#ifndef _LAZY_LAYERED_GRID_MAP_H_INCLUDED
#define _LAZY_LAYERED_GRID_MAP_H_INCLUDED

#include <memory>
#include <cmath>
#include <cstring>
#include <vector>
#include <array>
#include <algorithm>

#include "cell_occupancy_estimator.h"
#include "grid_cell.h"
#include "grid_map.h"
#include "../geometry_utils.h"
#include <iostream>

class LazyLayeredGridMap : public GridMap {
private:
  static constexpr unsigned TILE_NM = 9;
  static constexpr unsigned TILE_SIZE_BITS = 7;
  static constexpr unsigned TILE_SIZE = 1 << TILE_SIZE_BITS;
  static constexpr unsigned TILE_COORD_MASK = TILE_SIZE - 1;
private:
  struct Tile;
public:
  LazyLayeredGridMap(std::shared_ptr<GridCell> prototype)
    : GridMap{prototype, TILE_SIZE*TILE_NM, TILE_SIZE*TILE_NM}
    , _unknown_cell{prototype->clone()}
    , _tiles{TILE_NM*TILE_NM, std::make_shared<Tile>(_unknown_cell)} {}

  virtual GridCell &operator[](const DPnt2D& coord) override {
    //    if (!has_cell(cell_coord)) {
    //  return;
    //}
    std::shared_ptr<Tile> &tile = this->tile(coord);
    if (!tile) {
      tile = std::make_shared<Tile>(_unknown_cell);
    }
    if (1 < tile.use_count()) {
      tile.reset(new Tile{*tile});
    }

    std::shared_ptr<GridCell> &cell = tile->cell(coord);
    if (1 < cell.use_count()) {
      cell = cell->clone();
    }
    return *cell;
  }

  virtual const GridCell &operator[](const DPnt2D& coord) const override {
    //assert(has_cell(cell_coord));
    //if (!has_cell(cell_coord)) {
    //  return _unknown_cell->value();
    //}
    return *tile(coord)->cell(coord);
  }

  virtual bool has_cell(const DPnt2D& cell_coord) const {
    return 0 <= cell_coord.x && cell_coord.x < width() &&
           0 <= cell_coord.y && cell_coord.y < height();
  }

private: // map
  std::shared_ptr<Tile> &tile(const DPnt2D &c) const {
    return  _tiles[(c.y >> TILE_SIZE_BITS) * TILE_NM +
                   (c.x >> TILE_SIZE_BITS)];
  }

private: // types
  struct Tile {
    Tile(std::shared_ptr<GridCell> dflt) {
      std::fill(_cells.begin(), _cells.end(), dflt);
    }

    std::shared_ptr<GridCell> &cell(const DPnt2D& cell_coord) const {
      return _cells[(cell_coord.x & TILE_COORD_MASK) * TILE_SIZE +
                    (cell_coord.y & TILE_COORD_MASK)];
    }
    mutable std::array<std::shared_ptr<GridCell>, TILE_SIZE*TILE_SIZE> _cells;
  };

private: // fields
  std::shared_ptr<GridCell> _unknown_cell;
  mutable std::vector<std::shared_ptr<Tile>> _tiles;
};

#endif
