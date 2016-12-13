#ifndef _LAZY_TILED_GRID_MAP_H_INCLUDED
#define _LAZY_TILED_GRID_MAP_H_INCLUDED

#include <memory>
#include <cmath>
#include <cstring>
#include <vector>
#include <array>
#include <algorithm>
#include <tuple>

#include "cell_occupancy_estimator.h"
#include "grid_cell.h"
#include "grid_map.h"
#include "../geometry_utils.h"
#include <iostream>

class LazyTiledGridMap : public GridMap {
protected:
  static constexpr unsigned Tile_Size_Bits = 7;
  static constexpr unsigned Tile_Size = 1 << Tile_Size_Bits;
  static constexpr unsigned Tile_Coord_Mask = Tile_Size - 1;
protected:
  struct Tile;
public:
  LazyTiledGridMap(std::shared_ptr<GridCell> prototype,
                   const GridMapParams& params = MapValues::gmp)
    : GridMap{prototype, params}
    , _unknown_cell{prototype->clone()}
    , _unknown_tile{std::make_shared<Tile>(_unknown_cell)}
    , _tiles_nm_x{(GridMap::width() + Tile_Size - 1) / Tile_Size}
    , _tiles_nm_y{(GridMap::height() + Tile_Size - 1) / Tile_Size}
    , _tiles{_tiles_nm_x * _tiles_nm_y, _unknown_tile} {}

  GridCell &operator[](const DPnt2D& c) override {
    DPnt2D coord = external2internal(c);
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

  const GridCell &operator[](const DPnt2D& c) const override {
    DPnt2D coord = external2internal(c);
    return *tile(coord)->cell(coord);
  }

protected: // methods & types

  const std::shared_ptr<GridCell> unknown_cell() const { return _unknown_cell; }
  std::shared_ptr<Tile> unknown_tile() { return _unknown_tile; }

  std::tuple<unsigned, unsigned>
  extra_tiles_nm(int min, int val, int max) const {
    assert(min <= max);
    unsigned prepend_nm = 0, append_nm = 0;
    if (val < min) {
      prepend_nm = 1 + ((min - val) >> Tile_Size_Bits);
    } else if (max <= val) {
      append_nm = 1 + ((val - max) >> Tile_Size_Bits);
    }
    return std::make_tuple(prepend_nm, append_nm);
  }

  struct Tile {
    Tile(std::shared_ptr<GridCell> dflt) {
      std::fill(_cells.begin(), _cells.end(), dflt);
    }

    std::shared_ptr<GridCell> &cell(const DPnt2D& cell_coord) {
      return const_cast<std::shared_ptr<GridCell> &>(
        static_cast<const Tile*>(this)->cell(cell_coord));
    }

    const std::shared_ptr<GridCell> &cell(const DPnt2D& cell_coord) const {
      return _cells[(cell_coord.x & Tile_Coord_Mask) * Tile_Size +
                    (cell_coord.y & Tile_Coord_Mask)];
    }
  private:
    std::array<std::shared_ptr<GridCell>, Tile_Size*Tile_Size> _cells;
  };

private: // methods
  std::shared_ptr<Tile> &tile(const DPnt2D &c) const {
    return  _tiles[(c.y >> Tile_Size_Bits) * _tiles_nm_x +
                   (c.x >> Tile_Size_Bits)];
  }

private: // fields
  std::shared_ptr<GridCell> _unknown_cell;
  std::shared_ptr<Tile> _unknown_tile;
protected: // fields
  unsigned _tiles_nm_x, _tiles_nm_y;
  mutable std::vector<std::shared_ptr<Tile>> _tiles;
};

/* Unbounded implementation */

class UnboundedLazyTiledGridMap : public LazyTiledGridMap {
public:
  UnboundedLazyTiledGridMap(std::shared_ptr<GridCell> prototype,
      const GridMapParams& params = MapValues::gmp)
    : LazyTiledGridMap{prototype, params}
    , _origin{GridMap::origin()} {}

  GridCell &operator[](const DPnt2D& c) override {
    ensure_inside(c);
    return LazyTiledGridMap::operator[](c);
  }

  const GridCell &operator[](const DPnt2D& c) const override {
    if (!has_cell(c)) {
      return *unknown_cell();
    }

    return LazyTiledGridMap::operator[](c);
  }

  DiscretePoint2D origin() const override { return _origin; }

protected:

  bool ensure_inside(const DiscretePoint2D &c) {
    if (has_cell(c)) return false;

    DPnt2D coord = external2internal(c);
    unsigned prep_x = 0, app_x = 0, prep_y = 0, app_y = 0;
    std::tie(prep_x, app_x) = extra_tiles_nm(0, coord.x, width());
    std::tie(prep_y, app_y) = extra_tiles_nm(0, coord.y, height());

    unsigned new_tiles_nm_x = prep_x + _tiles_nm_x + app_x;
    unsigned new_tiles_nm_y = prep_y + _tiles_nm_y + app_y;
    assert(_tiles_nm_x <= new_tiles_nm_x && _tiles_nm_y <= new_tiles_nm_y);

    std::vector<std::shared_ptr<Tile>> new_tiles{new_tiles_nm_x*new_tiles_nm_y,
                                                 unknown_tile()};
    for (unsigned row_i = 0; row_i != _tiles_nm_y; ++row_i) {
      std::move(&_tiles[row_i * _tiles_nm_x],       // row begin
                &_tiles[(row_i + 1) * _tiles_nm_x], // row end
                &new_tiles[(prep_y + row_i)* new_tiles_nm_x + prep_x]);
    }

    _tiles_nm_x = new_tiles_nm_x;
    _tiles_nm_y = new_tiles_nm_y;
    std::swap(_tiles, new_tiles);
    set_height(_tiles_nm_y * Tile_Size);
    set_width(_tiles_nm_x * Tile_Size);
    _origin += DPnt2D(prep_x * Tile_Size, prep_y * Tile_Size);

    assert(has_cell(c));
    return true;
  }

private: // fields
    DiscretePoint2D _origin;
};

#endif
