#ifndef SLAM_CTOR_CORE_GRID_RASTERIZATION_H
#define SLAM_CTOR_CORE_GRID_RASTERIZATION_H

#include <vector>

#include "../geometry_utils.h"
#include "regular_squares_grid.h"

// FIXME: GridRasterizedRectangle{grid, grid.world_cell_bounds(cell)}
//          != { cell }
// WA: GridRasterizedRectangle{grid, grid.world_cell_bounds(cell), false}
//          == { cell }
class GridRasterizedRectangle {
public:
  /* IDEA:
   *
   * auto left_bot = grid.world_to_cell(area.left(), area.bot());
   * auto right_top = grid.world_to_cell(area.right(), area.top());
   * for (int y = left_bot.y; y <= right_top.y; ++y) {
   *   for (int x = left_bot.x; x <= right_top.x; ++x) {
   *     // action
   *   }
   * }
  */
  GridRasterizedRectangle(const RegularSquaresGrid &grid,
                          const LightWeightRectangle &rect,
                          bool include_border = true) {
    double offset = include_border ? 0 : 1e-9;
    if (rect.area() == RegularSquaresGrid::Dbl_Inf) {
      _lb = grid.internal2external({0, 0});
      _rt = grid.internal2external({grid.width() - 1, grid.height() - 1});
    } else {
      _lb = grid.world_to_cell(rect.left() + offset, rect.bot() + offset);
      _rt = grid.world_to_cell(rect.right() - offset, rect.top() - offset);
    }
    _y = _lb.y;
    _x = _lb.x;
  }

  bool has_next() {
    //NB: aligned top/right border is a part of a nearby cell
    //    according to implemented geometry
    return _y <= _rt.y && _x <= _rt.x;
  }

  RegularSquaresGrid::Coord next() {
    auto area_id = RegularSquaresGrid::Coord{_x, _y};
    ++_x;
    if (_rt.x < _x) {
      _x = _lb.x;
      ++_y;
    }
    return area_id;
  }

  auto to_vector() && {
    auto result = std::vector<RegularSquaresGrid::Coord>{};
    while (has_next()) { result.push_back(next()); }
    return result;
  }

private:
  RegularSquaresGrid::Coord _lb, _rt;
  int _x, _y;
};

#endif
