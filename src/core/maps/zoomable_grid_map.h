#ifndef SLAM_CTOR_CORE_ZOOMABLE_GRID_MAP_H_INCLUDED
#define SLAM_CTOR_CORE_ZOOMABLE_GRID_MAP_H_INCLUDED

#include <memory>
#include <cassert>
#include <limits>
#include <utility>

#include "grid_map.h"

template <typename T>
class ZoomableGridMap : public GridMap {
private: // type aliases
  using ZoomedMapCache = std::vector<std::unique_ptr<GridMap>>;
private: // classes declarations
  class ZoomableGridCellProxy;
private: // consts
  static constexpr int Coarsest_Map_W = 1, Coarsest_Map_H = 1;
public:

  ZoomableGridMap(std::shared_ptr<GridCell> prototype,
                  const GridMapParams& params = MapValues::gmp)
    : GridMap{prototype, params}
    , _zoomed_map_cache{std::make_shared<ZoomedMapCache>()} {

    // finest map
    _zoomed_map_cache->push_back(std::make_unique<T>(prototype, params));
    // coarsest map
    auto coarsest_mp = GridMapParams{Coarsest_Map_W, Coarsest_Map_H,
                                     std::numeric_limits<double>::infinity()};
    _zoomed_map_cache->push_back(std::make_unique<T>(prototype, coarsest_mp));
    ensure_zcache_is_continuous();
  }

  ZoomableGridMap(const ZoomableGridMap&) = delete;
  ZoomableGridMap& operator=(const ZoomableGridMap&) = delete;
  ZoomableGridMap(ZoomableGridMap&&) = default;
  ZoomableGridMap& operator=(ZoomableGridMap&&) = default;

  static constexpr unsigned finest_zoom_level() { return 0; }
  unsigned coarsest_zoom_level() const { return zoom_levels_nm() - 1; }

  unsigned zoom_levels_nm() const {
    ensure_zcache_is_continuous();
    return _zoomed_map_cache->size();
  }

  void set_zoom_level(unsigned zoom_level) {
    assert(zoom_level < zoom_levels_nm());
    _zoom_level = zoom_level;
  }

  //----------------------------------------------------------------------------
  // GridMap overrides

  int width() const override { return active_map().width(); }
  int height() const override { return active_map().height(); }
  double scale() const override { return active_map().scale(); }
  DiscretePoint2D origin() const override { return active_map().origin(); }

  const GridCell& operator[](const DPnt2D& coord) const override {
    return active_map()[coord];
  }

  // non-const op[] must return a GridCell reference according to its iface
  // WA: use a pin to track proxy's lifetime
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
    _proxy_pin.reset(new ZoomableGridCellProxy{coord, _zoom_level,
                                               _zoomed_map_cache});
    return *_proxy_pin;
  }

private:

  const GridMap& map(unsigned zoom_level) const {
    return *(*_zoomed_map_cache)[zoom_level];
  }

  const GridMap& active_map() const {return map(_zoom_level); }

  GridMap& active_map() {
    return const_cast<GridMap&>(
      static_cast<const decltype(this)>(this)->active_map());
  }

  void ensure_zcache_is_continuous() const {
    static const int PC_W_Target = Coarsest_Map_W * map_scale_factor(),
                     PC_H_Target = Coarsest_Map_H * map_scale_factor();

    const GridMap& pre_coarsest_map = map(_zoomed_map_cache->size() - 2);
    int pc_w = pre_coarsest_map.width(), pc_h = pre_coarsest_map.height();
    double pc_scale = pre_coarsest_map.scale();

    if (pc_w <= PC_W_Target && pc_h <= PC_H_Target) { return; }

    pc_w = ge_2_pow(pc_w), pc_h = ge_2_pow(pc_h);
    // more cache levels have to be added
    while (PC_W_Target < pc_w || PC_H_Target < pc_h) {
      pc_w = std::max(PC_W_Target, int(std::ceil(pc_w / map_scale_factor())));
      pc_h = std::max(PC_H_Target, int(std::ceil(pc_h / map_scale_factor())));
      pc_scale *= map_scale_factor();

      auto map_params = GridMapParams{pc_w, pc_h, pc_scale};
      _zoomed_map_cache->insert((_zoomed_map_cache->rbegin() + 1).base(),
                                std::make_unique<T>(cell_prototype(),
                                                    map_params));
    }
  }

  static double map_scale_factor() {
    return 2;
  }

private: // classes

  class ZoomableGridCellProxy : public GridCell {
  public:
    ZoomableGridCellProxy(const DPnt2D& coord, unsigned zoom_level,
                          std::shared_ptr<ZoomedMapCache> map_cache)
      : GridCell{Occupancy{0, 0}}
      , _zoom_level{zoom_level}, _coord{coord}, _map_cache{map_cache} {
    }

    unsigned zoom_level() { return _zoom_level; }
    const DPnt2D& coord() { return _coord; }

    //--------------------------------------------------------------------------
    // GridCell overrides

    const Occupancy& occupancy() const {
      return cell(_zoom_level, _coord).occupancy();
    }

    std::unique_ptr<GridCell> clone() const override {
      return std::make_unique<ZoomableGridCellProxy>(*this);
    }

    void operator+=(const AreaOccupancyObservation &aoo) override {
      // LADO: update of several cells if "non-unzoomed" cell is updated
      assert(_zoom_level == finest_zoom_level());

      // * Update back cell
      GridCell& master = cell(_zoom_level, _coord);
      master += aoo;


      // * Update coarser levels' cells
      const Point2D vphys_coord = active_map(_zoom_level).cell_to_world(_coord);
      for (unsigned zl = _zoom_level + 1; zl < _map_cache->size(); ++zl) {
        auto coarser_master_coord = active_map(zl).world_to_cell(vphys_coord);
        GridCell &coarser_master = cell(zl, coarser_master_coord);
        // LADO: move to Bounding Strategy
        if (double(master) <= double(coarser_master)) {
          break;
        }
        coarser_master = master;
      }
      // LADO: Update finer levels
    }

    double discrepancy(const AreaOccupancyObservation &aoo) const override {
      return cell(_zoom_level, _coord).discrepancy(aoo);
    }

  private:

    GridMap& active_map(unsigned zoom_lvl) const {
      return *(*_map_cache)[zoom_lvl];
    }

    const GridCell& cell(unsigned zoom_lvl, const DPnt2D &coord) const {
      const GridMap& map = static_cast<decltype(map)>(active_map(zoom_lvl));
      // NB: "op[] const" and "op[]" may have different semantics for GridMap
      return map[coord];
    }
    GridCell& cell(unsigned zoom_lvl, const DPnt2D &coord) {
      return active_map(zoom_lvl)[coord];
    }
  private:
    unsigned _zoom_level;
    DPnt2D _coord;
    std::shared_ptr<ZoomedMapCache> _map_cache; // owner - outer class
  };

private:
  // TODO: several pins
  std::shared_ptr<ZoomableGridCellProxy> _proxy_pin;
  unsigned _zoom_level = finest_zoom_level();
  mutable std::shared_ptr<ZoomedMapCache> _zoomed_map_cache;
};


#endif // header guard
