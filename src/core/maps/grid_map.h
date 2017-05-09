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

class GridMap;

class GridMapUpdateObserver {
public:
  virtual void on_grid_map_update(const GridMap& map,
                                  const RegularSquaresGrid::Coord &area_id) = 0;
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
  virtual void update(const Coord& area_id,
                      const AreaOccupancyObservation &aoo) {
    auto const_this = static_cast<const decltype(this)>(this);
    auto &area = const_cast<GridCell&>((*const_this)[area_id]);
    update_area_occupancy(area_id, area, aoo);
  }
  virtual const GridCell &operator[](const Coord& coord) const = 0;

  void observe_updates(std::shared_ptr<GridMapUpdateObserver> obs) const {
    _update_observers.push_back(obs);
  }

protected:
  inline void update_area_occupancy(const Coord& area_id, GridCell &area,
                                    const AreaOccupancyObservation &aoo) {
    area += aoo;
    for (auto &obs : _update_observers) {
      if (auto obs_ptr = obs.lock()) {
        obs_ptr->on_grid_map_update(*this, area_id);
      }
    }
  }

  std::shared_ptr<GridCell> cell_prototype() const { return _cell_prototype; }
private: // fields
  std::shared_ptr<GridCell> _cell_prototype;
  mutable std::vector<std::weak_ptr<GridMapUpdateObserver>> _update_observers;
};

#endif
