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
  // TODO: make iterable
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
                          const LightWeightRectangle &r,
                          bool include_border = true) {
    double offset = include_border ? 0 : 1e-9;
    bool area_is_infin = r.area() == RegularSquaresGrid::Dbl_Inf,
         area_is_empty = r.area() == 0;
    double scale = grid.scale();
    if (!area_is_empty && !area_is_infin) {
      _lb = grid.world_to_cell(r.left() + offset, r.bot() + offset, scale);
      _rt = grid.world_to_cell(r.right() - offset, r.top() - offset, scale);
    } else if (area_is_empty) {
      // TODO: do not pass scale as an argument. This causes slight slow down.
      //       Find out the reason.
      _rt = _lb = grid.world_to_cell(r.left(), r.bot());
    } else if (area_is_infin) {
      _lb = grid.internal2external({0, 0});
      _rt = grid.internal2external({grid.width() - 1, grid.height() - 1});
    } else {
      assert("BUG: rectangle area is both empty and infinite");
    }
    _y = _lb.y;
    _x = _lb.x;
  }

  bool has_next() {
    //NB: aligned top/right border is a part of a nearby cell
    //    according to implemented geometry
    return _x <= _rt.x && _y <= _rt.y;
  }

  RegularSquaresGrid::Coord next() {
    auto area_id = RegularSquaresGrid::Coord{_x, _y};
    ++_y;
    if (_rt.y < _y) {
      _y = _lb.y;
      ++_x;
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
