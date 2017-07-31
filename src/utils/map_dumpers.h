#ifndef SLAM_CTOR_UTIL_MAP_DUMPERS
#define SLAM_CTOR_UTIL_MAP_DUMPERS

#include <fstream>
#include <string>

#include "../core/math_utils.h"
#include "../core/states/state_data.h"
#include "../core/maps/grid_map.h"

template <typename GridMapType>
class GridMapToPgmDumber : public WorldMapObserver<GridMapType> {
private:
  static constexpr short Max_Intensity = 255;
  using IntensityType = char;

private:
  class PgmHeader {
  public:
    PgmHeader(int w, int h, int max_v)
      : _width{w}, _height{h}, _max_val{max_v} {}

    void write(std::ostream &os) const {
      os.write(_magic, sizeof(_magic));
      write_whitespace(os);
      write_int(os, _width);
      write_whitespace(os);
      write_int(os, _height);
      write_whitespace(os);
      write_int(os, _max_val);
    }
  private:

    void write_whitespace(std::ostream &os) const {
      char new_line = '\n';
      os.write(&new_line, sizeof(new_line));
    }

    void write_int(std::ostream &os, int n) const {
      auto str = std::to_string(n);
      os.write(str.c_str(), str.size());
    }

  private:
    char _magic[2] = {'P', '5'};
    int _width, _height, _max_val;
  };

public:
  GridMapToPgmDumber(const std::string &base_fname)
    : _base_fname{base_fname + "_"}
    , _id{0} {}

  void on_map_update(const GridMapType &map) override {
    auto dst = std::ofstream{_base_fname + std::to_string(_id) + ".pgm",
                             std::ios::binary | std::ios::out};
    dump_map(dst, map);
    dst.close();
    ++_id;
  }

  void dump_map(std::ofstream &os, const GridMapType &map) {
    auto w = map.width(), h = map.height();
    auto origin = map.origin();
    // write pgm header
    PgmHeader{w, h, Max_Intensity}.write(os);
    char new_line = '\n';
    os.write(&new_line, sizeof(new_line));

    // write map content
    using AreaId = typename GridMapType::Coord;
    auto area_id = AreaId{};

    int val_nm = 0;
    for (area_id.y = h - origin.y - 1; -origin.y <= area_id.y; --area_id.y) {
      for (area_id.x = -origin.x; area_id.x < w - origin.x; ++area_id.x) {
        auto value = 1.0 - std::max(0.0, std::min(1.0, map.occupancy(area_id)));
        //assert(are_ordered(0, value, 1));
        auto intensity = static_cast<IntensityType>(Max_Intensity * value);
        os.write(&intensity, sizeof(intensity));
        ++val_nm;
      }
    }
    assert(val_nm == w * h); // sanity
  }
private:
  std::string _base_fname;
  unsigned long long _id;
};

#endif
