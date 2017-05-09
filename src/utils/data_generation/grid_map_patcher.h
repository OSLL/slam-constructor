#ifndef SLAM_CTOR_UTILS_DG_GRID_MAP_PATCHER_INCLUDED
#define SLAM_CTOR_UTILS_DG_GRID_MAP_PATCHER_INCLUDED

#include <istream>
#include <string>
#include <vector>
#include <cmath>
#include <tuple>

#include "../../core/maps/grid_map.h"

class GridMapPatcher {
private:

  struct GridMapPatch {
    GridMapPatch(const DiscretePoint2D &c, const Occupancy &occ)
      : coord{c}, occupancy{occ} {}
    DiscretePoint2D coord;
    Occupancy occupancy;
  };

  using GMPatches = std::vector<GridMapPatch>;

public:

  template<typename MapType>
  void apply_text_raster(MapType &dst_map, std::istream &src_raster,
                         const DiscretePoint2D &dst_offset,
                         int w_zoom = 1, int h_zoom = 1) {
    apply_text_raster(dst_map, src_raster, false, dst_offset, w_zoom, h_zoom);
  }

  template<typename MapType>
  void apply_text_raster(MapType &dst_map, std::istream &src_raster,
                         int w_zoom = 1, int h_zoom = 1) {
    apply_text_raster(dst_map, src_raster, true, {}, w_zoom, h_zoom);
  }

private: // methods

  Occupancy char_to_occupancy(char ch) {
    return Occupancy{ch == ' ' ? 0.0 : 1.0, 1.0};
  }

  template<typename MapType>
  void apply_text_raster(MapType &dst_map, std::istream &src_raster,
                         bool autoalign, const DiscretePoint2D &dst_offset,
                         int w_zoom, int h_zoom) {
    assert(0 < w_zoom && 0 < h_zoom);
    auto patches = patches_by_text_raster(src_raster, w_zoom, h_zoom);
    auto offset = autoalign ? DiscretePoint2D{-w_zoom * _curr_raster_w / 2,
                                              h_zoom * _curr_raster_h / 2}
                            : dst_offset;
    apply_patches(dst_map, offset, patches);
  }

  GMPatches patches_by_text_raster(std::istream &raster,
                                   int w_scale, int h_scale) {
    GMPatches patches;
    _curr_raster_w = _curr_raster_h = 0;

    auto world_coord = DiscretePoint2D{0, 0};
    std::string raw_map_row;
    while (raster.good()) {
      std::getline(raster, raw_map_row);
      if (raw_map_row.size() == 0) {
        break; // Stop at empty line
      }

      for (int row_i = 0; row_i < h_scale; ++row_i) {
        world_coord.x = 0;
        for (auto ch : raw_map_row) {
          auto occupancy = char_to_occupancy(ch);
          for (int col_i = 0; col_i < w_scale; ++col_i) {
            patches.emplace_back(world_coord, occupancy);
            world_coord.x += 1;
          }
        }
        world_coord.y -= 1;
      } // for row_i
      _curr_raster_w = std::max(_curr_raster_w,
                                static_cast< int>(raw_map_row.size()));
      _curr_raster_h += 1;
    } // while (raster)
    return patches;
  }

  template<typename MapType>
  void apply_patches(MapType &dst_map,
                     const DiscretePoint2D &offset, const GMPatches &patches) {

    auto aoo = AreaOccupancyObservation{true, Occupancy{0, 0},
                                        Point2D{0, 0}, 1 /* quality */};
    for (auto &upd_req : patches) {
      aoo.occupancy = upd_req.occupancy;
      dst_map.update(upd_req.coord + offset, aoo);
    }
  }

private: // fields
  int _curr_raster_w, _curr_raster_h;
};

#endif
