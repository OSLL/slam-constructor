#ifndef SLAM_CTOR_CORE_RESCALABLE_CACHING_GRID_MAP_H
#define SLAM_CTOR_CORE_RESCALABLE_CACHING_GRID_MAP_H

#include <memory>
#include <cassert>
#include <limits>
#include <utility>
#include <iterator>
#include <algorithm>

#include "grid_cell.h"
#include "grid_map.h"
#include "grid_rasterization.h"
#include "../serialization.h"

template <typename BackGridMap>
class RescalableCachingGridMap : public GridMap {
private: // type aliases
  using MapCache = std::vector<std::unique_ptr<GridMap>>;
private: // consts
  static constexpr int Coarsest_Map_W = 1, Coarsest_Map_H = 1;
public: // type aliases
  using BackMapT = BackGridMap;
public: // consts
  static constexpr unsigned Map_Scale_Factor = 2;
public:
  RescalableCachingGridMap(std::shared_ptr<GridCell> prototype,
                           const GridMapParams& params = MapValues::gmp)
    : GridMap{prototype, params}
    , _map_cache{std::make_shared<MapCache>()} {

    // the finest map
    _map_cache->push_back(std::make_unique<BackGridMap>(prototype, params));
    // the coarsest map
    auto coarsest_mp = GridMapParams{Coarsest_Map_W, Coarsest_Map_H,
                                     std::numeric_limits<double>::infinity()};
    auto coarsest_map = std::make_unique<BackGridMap>(prototype, coarsest_mp);
    _map_cache->push_back(std::move(coarsest_map));
    ensure_map_cache_is_continuous();

    set_scale_id(finest_scale_id());
  }

  RescalableCachingGridMap(const RescalableCachingGridMap&) = delete;
  RescalableCachingGridMap& operator=(const RescalableCachingGridMap&) = delete;
  RescalableCachingGridMap(RescalableCachingGridMap&&) = default;
  RescalableCachingGridMap& operator=(RescalableCachingGridMap&&) = default;

  //----------------------------------------------------------------------------
  // API for manual scale management.

  unsigned scales_nm() const {
    ensure_map_cache_is_continuous();
    return _map_cache->size();
  }

  unsigned scale_id() const { return _scale_id; }

  static constexpr unsigned finest_scale_id() { return 0; }
  unsigned coarsest_scale_id() const { return scales_nm() - 1; }

  void set_scale_id(unsigned scale_id) {
    assert(scale_id < scales_nm());
    _scale_id = scale_id;
    _active_map = &map(_scale_id);
  }

  //----------------------------------------------------------------------------
  // RegularSquaresGrid overrides

  Coord origin() const override { return active_map().origin(); }
  int width() const override { return active_map().width(); }
  int height() const override { return active_map().height(); }
  double scale() const override { return active_map().scale(); }
  bool has_cell(const Coord &c) const override {
    return active_map().has_cell(c);
  }

  void rescale(double target_scale) override {
    ensure_map_cache_is_continuous();

    // TODO: replace the linear probing
    unsigned scale_id = finest_scale_id();
    while (1) {
      if (target_scale <= map(scale_id).scale()) {
        break;
      }
      ++scale_id;
    }
    set_scale_id(scale_id);
  }

  //----------------------------------------------------------------------------
  // GridMap overrides

  const GridCell& operator[](const Coord &coord) const override {
    return active_map()[coord];
  }

  void update(const Coord &area_id,
              const AreaOccupancyObservation &aoo) override {
    active_map().update(area_id, aoo);
    on_area_update(area_id);
  }

  void reset(const Coord &area_id,
             const GridCell &area) override {
    active_map().reset(area_id, area);
    on_area_update(area_id);
  }

  // = Serialization
  // Q: Do we need to save currest scale_id?
  std::vector<char> save_state() const override {
    auto map_s = Serializer{};
    assert(coarsest_scale_id() - finest_scale_id() == scales_nm() - 1);
    for (auto id = finest_scale_id(); id <= coarsest_scale_id(); id++) {
      auto map_data = map(id).save_state();
      map_s << map_data.size();
      map_s.append(map_data);
    }
    return map_s.result();
  }

  void load_state(const std::vector<char>& data) override {
    auto map_d = Deserializer{data};
    _map_cache->clear();
    while (map_d.pos() != data.size()) {
      decltype(data.size()) chunk_size;
      map_d >> chunk_size;
      auto ptr = data.begin() + map_d.pos();
      assert(ptr + chunk_size <= data.end() && "Unable to load map");
      auto map_chunk = std::vector<char>{};
      std::copy(ptr, ptr + chunk_size, std::back_inserter(map_chunk));
      map_d.inc_pose(chunk_size);

      BackGridMap curr_map{cell_prototype(), {0, 0, 0}};
      curr_map.load_state(map_chunk);
      _map_cache->push_back(std::make_unique<BackGridMap>(std::move(curr_map)));
    }
    ensure_map_cache_is_continuous();
    set_scale_id(finest_scale_id());
  }

protected:

  // coraser areas update policy
  virtual void on_area_update(const AreaId &area_id) {}

  const GridMap& map(unsigned scale_id) const {
    return *(*_map_cache)[scale_id];
  }

  GridMap& map(unsigned scale_id) {
    return const_cast<GridMap&>(
      static_cast<const RescalableCachingGridMap&>(*this).map(scale_id));
  }

  const GridMap& active_map() const {
    return *_active_map;
  }

  GridMap& active_map() {
    return const_cast<GridMap&>(
      static_cast<const RescalableCachingGridMap&>(*this).active_map());
  }

  void ensure_map_cache_is_continuous() const {
    if (_map_cache->size() < 2) { return; }
    static const int PC_W_Target = Coarsest_Map_W * Map_Scale_Factor,
                     PC_H_Target = Coarsest_Map_H * Map_Scale_Factor;

    const GridMap& pre_coarsest_map = map(_map_cache->size() - 2);
    int pc_w = pre_coarsest_map.width(), pc_h = pre_coarsest_map.height();
    double pc_scale = pre_coarsest_map.scale();

    if (pc_w <= PC_W_Target && pc_h <= PC_H_Target) { return; }

    pc_w = ge_pow<Map_Scale_Factor>(pc_w);
    pc_h = ge_pow<Map_Scale_Factor>(pc_h);
    // more cache levels have to be added
    while (PC_W_Target < pc_w || PC_H_Target < pc_h) {
      pc_w = std::max(PC_W_Target, int(std::ceil(pc_w / Map_Scale_Factor)));
      pc_h = std::max(PC_H_Target, int(std::ceil(pc_h / Map_Scale_Factor)));
      pc_scale *= Map_Scale_Factor;

      auto map_params = GridMapParams{pc_w, pc_h, pc_scale};
      auto map = std::make_unique<BackGridMap>(cell_prototype(), map_params);
      _map_cache->insert((_map_cache->rbegin() + 1).base(), std::move(map));
    }
  }

private:
  GridMap *_active_map = nullptr;
  unsigned _scale_id = -1;
  mutable std::shared_ptr<MapCache> _map_cache;
};

// a RAII for const grid map rescaling
class SafeRescalableMap {
public:
 SafeRescalableMap(const GridMap &map)
   : _vanilla_scale{map.scale()}, _map{const_cast<GridMap&>(map)} {}
  ~SafeRescalableMap() { _map.rescale(_vanilla_scale); }
  SafeRescalableMap(const SafeRescalableMap&) = delete;
  SafeRescalableMap& operator=(const SafeRescalableMap&) = delete;
  SafeRescalableMap(SafeRescalableMap&&) = delete;
  SafeRescalableMap& operator=(SafeRescalableMap&&) = delete;

  operator GridMap&() { return _map; }
private:
  double _vanilla_scale;
  GridMap &_map;
};

#endif // header guard
