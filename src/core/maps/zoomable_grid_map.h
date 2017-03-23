#ifndef SLAM_CTOR_CORE_ZOOMABLE_GRID_MAP_H_INCLUDED
#define SLAM_CTOR_CORE_ZOOMABLE_GRID_MAP_H_INCLUDED

#include <memory>
#include <cassert>

#include "grid_map.h"

template <typename T>
class ZoomableGridMap : public GridMap {
private:
  using ZoomedMapCache = std::vector<std::unique_ptr<GridMap>>;
private:
  class ZoomableGridCellProxy;
public:
  static constexpr unsigned Unzoomed_Map_Level = 0;
public:

  ZoomableGridMap(std::shared_ptr<GridCell> prototype,
                  const GridMapParams& params = MapValues::gmp)
    // TODO: height, width should be taken from the base map
    : GridMap{prototype, params}
    , _zoomed_map_cache{std::make_shared<ZoomedMapCache>()} {

    _zoomed_map_cache->push_back(std::make_unique<T>(prototype, params));
    ensure_zoom_cache_is_valid();
  }

  ZoomableGridMap(const ZoomableGridMap&) = delete;
  ZoomableGridMap& operator=(const ZoomableGridMap&) = delete;
  ZoomableGridMap(ZoomableGridMap&&) = default;
  ZoomableGridMap& operator=(ZoomableGridMap&&) = default;

  int width() const override {
    return (*_zoomed_map_cache)[_zoom_level]->width();
  }
  int height() const override {
    return (*_zoomed_map_cache)[_zoom_level]->height();
  }
  double scale() const override {
    return (*_zoomed_map_cache)[_zoom_level]->scale();
  }

  DiscretePoint2D origin() const override {
    return (*_zoomed_map_cache)[_zoom_level]->origin();
  }

  ZoomableGridCellProxy& operator[](const DPnt2D& coord) override {
    if (_proxy_pin.get() != nullptr) {
      if (_proxy_pin->zoom_level() == _zoom_level &&
          _proxy_pin->coord() == coord) {
        return *_proxy_pin; // the cell has already been pinned
      } else if (_proxy_pin.use_count() == 1) {
        // ok, the only user is the proxy
      } else {
        assert(0 && "Try to access for modification two cells simultaneously");
      }
    }
    _proxy_pin.reset(
      new ZoomableGridCellProxy{_zoom_level, coord, _zoomed_map_cache});
    return *_proxy_pin;
  }

  const GridCell &operator[](const DPnt2D& coord) const override {
    const GridMap& active_map = *(*_zoomed_map_cache)[_zoom_level];
    return active_map[coord];
  }

  void set_zoom_level(unsigned zoom_level) {
    assert(zoom_level < zoom_levels_nm());
    _zoom_level = zoom_level;
  }

  unsigned zoom_levels_nm() {
    ensure_zoom_cache_is_valid();
    return _zoomed_map_cache->size();
  }

private:

  void ensure_zoom_cache_is_valid() {
    GridMap* the_coarsest_map = _zoomed_map_cache->back().get();
    if (the_coarsest_map->width() == 1 && the_coarsest_map->height() == 1) {
      return;
    }

    int w = the_coarsest_map->width(), h = the_coarsest_map->height();
    double cell_size = the_coarsest_map->scale();

    while (w != 1 || h != 1) {
      w = std::max(1, int(std::ceil(1.0 * w / map_scale_factor())));
      h = std::max(1, int(std::ceil(1.0 * h / map_scale_factor())));
      cell_size *= map_scale_factor();

      _zoomed_map_cache->push_back(
        std::make_unique<T>(cell_prototype(), GridMapParams{w, h, cell_size})
      );
    }
  }

  static int map_scale_factor() {
    return 2;
  }

private:
  class ZoomableGridCellProxy : public GridCell {
  public:
    ZoomableGridCellProxy(unsigned zoom_level, const DPnt2D& coord,
                          std::shared_ptr<ZoomedMapCache> map_cache)
      : GridCell{Occupancy{0, 0}}
      , _zoom_level{zoom_level}, _coord{coord}, _map_cache{map_cache} {
      _occupancy = master_cell().occupancy();
    }

    std::unique_ptr<GridCell> clone() const override {
      return std::make_unique<ZoomableGridCellProxy>(*this);
    }

    void operator+=(const AreaOccupancyObservation &aoo) override {
      // TODO: update of several cells
      assert(_zoom_level == Unzoomed_Map_Level);

      GridCell& master = master_cell();
      master += aoo;
      _occupancy = master.occupancy();

      const GridMap& active_map = *(*_map_cache)[_zoom_level];
      const Point2D vphys_coord = {(_coord.x + 0.5) * active_map.scale(),
                                   (_coord.y + 0.5) * active_map.scale()};
      // Update upper cells
      for (unsigned zl = _zoom_level + 1; zl < _map_cache->size(); ++zl) {
        // TODO CHECK: assert inside zoomed out map
        GridCell &zo_master = cell(zl, zoom_out(vphys_coord, zl));
        // TODO: move to policy
        if (double(master) < double(zo_master)) {
          break;
        }
        zo_master = master;
      }
      // TODO: update lower levels
    }

    double discrepancy(const AreaOccupancyObservation &aoo) const override {
      return master_cell().discrepancy(aoo);
    }

    unsigned zoom_level() { return _zoom_level; }
    const DPnt2D& coord() { return _coord; }
  private:
    // NB: "op[] const" and "op[]" may have different semantics for GridMap
    const GridCell& cell(unsigned zoom_lvl, const DPnt2D &coord) const {
      const GridMap& active_map = *(*_map_cache)[zoom_lvl];
      return active_map[coord];
    }
    GridCell& cell(unsigned zoom_lvl, const DPnt2D &coord) {
      return (*(*_map_cache)[zoom_lvl])[coord];
    }
    const GridCell& master_cell() const {
      return cell(_zoom_level, _coord);
    }
    GridCell& master_cell() {
      return cell(_zoom_level, _coord);
    }

    DPnt2D zoom_out(const Point2D& ph_pnt, unsigned zoom_lvl) {
      // TODO: scailing, code duplication -> move zoom out to strategy
      const GridMap& active_map = *(*_map_cache)[zoom_lvl];
      if (zoom_lvl != _map_cache->size() - 1) {
        return active_map.world_to_cell(ph_pnt);
      }

      // Treat last map as single pixel with origin at scale-center
      return active_map.world_to_cell(
        Point2D{ph_pnt.x + active_map.scale() / 2,
                ph_pnt.y + active_map.scale() / 2});
    }
  private:
    unsigned _zoom_level;
    DPnt2D _coord;
    std::shared_ptr<ZoomedMapCache> _map_cache; // owner - outer class
  };

private:
  // TODO: several pins
  std::shared_ptr<ZoomableGridCellProxy> _proxy_pin;
  unsigned _zoom_level = Unzoomed_Map_Level;
  std::shared_ptr<ZoomedMapCache> _zoomed_map_cache;
};


#endif // header guard
