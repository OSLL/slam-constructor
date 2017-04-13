#ifndef SLAM_CTOR_CORE_GRID_MAP_H_INCLUDED
#define SLAM_CTOR_CORE_GRID_MAP_H_INCLUDED

#include <memory>

#include "regular_squares_grid.h"
#include "grid_cell.h"

struct GridMapParams {
  const int width_cells, height_cells;
  const double meters_per_cell;
};

struct MapValues {
  static constexpr int width = 1000;
  static constexpr int height = 1000;
  static constexpr double meters_per_cell = 0.1;
  static constexpr GridMapParams gmp{width, height, meters_per_cell};
};

class GridMap : public RegularSquaresGrid {
public:
  GridMap(std::shared_ptr<GridCell> prototype,
          const GridMapParams& params = MapValues::gmp)
    : RegularSquaresGrid{params.width_cells, params.height_cells,
                         params.meters_per_cell}
    , _cell_prototype{prototype} {}

  std::unique_ptr<GridCell> new_cell() const {
    return _cell_prototype->clone();
  }

  // GridCell access
  // NB: use update/reset to modify cells instead of operator[]
  //     to prevent proxies usage in descendants that are interested in
  //     modifications.
  virtual void update(const Coord &area_id,
                      const AreaOccupancyObservation &aoo) {
    auto const_this = static_cast<const decltype(this)>(this);
    auto &area = const_cast<GridCell&>((*const_this)[area_id]);
    area += aoo;
  }

  virtual void reset(const Coord &area_id, const GridCell &new_area) {
    auto const_this = static_cast<const decltype(this)>(this);
    auto &area = const_cast<GridCell&>((*const_this)[area_id]);
    area = new_area;
  }

  virtual const GridCell &operator[](const Coord& coord) const = 0;

  virtual void load_state(const std::vector<char>&) {}
  virtual std::vector<char> save_state() const {
      return std::vector<char>();
  }

protected:

  std::shared_ptr<GridCell> cell_prototype() const { return _cell_prototype; }
private: // fields
  std::shared_ptr<GridCell> _cell_prototype;
};

#endif
