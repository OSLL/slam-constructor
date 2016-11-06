#ifndef _LAZY_LAYERED_GRID_MAP_H_INCLUDED
#define _LAZY_LAYERED_GRID_MAP_H_INCLUDED

#include <memory>
#include <cmath>
#include <cstring>
#include <vector>
#include <array>
#include <algorithm>

#include "cell_occupancy_estimator.h"
#include "grid_cell_factory.h"
#include "grid_map.h"
#include "../geometry_utils.h"
#include <iostream>

class LazyLayeredGridMap : public GridMap {
public:
  using DPnt = DiscretePoint2D;
private:
  static constexpr unsigned TILE_NM = 9;
  static constexpr unsigned TILE_SIZE_BITS = 7;
  static constexpr unsigned TILE_SIZE = 1 << TILE_SIZE_BITS;
  static constexpr unsigned TILE_COORD_MASK = TILE_SIZE - 1;
private:
  struct Tile;
public:
  LazyLayeredGridMap(std::shared_ptr<GridCellFactory> cell_factory)
    : GridMap{cell_factory, TILE_SIZE*TILE_NM, TILE_SIZE*TILE_NM}
    , _unknown_cell{cell_factory->create_cell()}
    , _tiles{TILE_NM*TILE_NM, std::make_shared<Tile>(_unknown_cell)} {}

  virtual void update_cell(const DPnt& cell_coord,
      const GridCellValue &new_value, double quality = 1.0) {
    //    if (!has_cell(cell_coord)) {
    //  return;
    //}
    std::shared_ptr<Tile> &tile = get_tile(cell_coord);
    if (!tile) {
      tile = std::make_shared<Tile>(_unknown_cell);
    }
    if (1 < tile.use_count()) {
      Tile *cloned_tile = new Tile{_unknown_cell};
      *cloned_tile = *tile;
      tile.reset(cloned_tile);
    }

    std::shared_ptr<GridCell> &cell = tile->get_cell(cell_coord);
    if (1 < cell.use_count()) {
      cell = cell->clone();
    }

    cell->set_value(new_value, quality);
  }

  virtual const GridCellValue &operator[](const DPnt& cell_coord) const {
    //assert(has_cell(cell_coord));
    //if (!has_cell(cell_coord)) {
    //  return _unknown_cell->value();
    //}
    return get_tile(cell_coord)->get_cell(cell_coord)->value();
  }

  virtual bool has_cell(const DPnt& cell_coord) const {
    return 0 <= cell_coord.x && cell_coord.x < width() &&
           0 <= cell_coord.y && cell_coord.y < height();
  }

private: // map
  std::shared_ptr<Tile> &get_tile(const DPnt &c) const {
    return  _tiles[(c.y >> TILE_SIZE_BITS) * TILE_NM +
                   (c.x >> TILE_SIZE_BITS)];
  }

private: // types
  struct Tile {
    Tile(std::shared_ptr<GridCell> dflt) {
      std::fill(_cells.begin(), _cells.end(), dflt);
    }

    std::shared_ptr<GridCell> &get_cell(const DPnt& cell_coord) const {
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
