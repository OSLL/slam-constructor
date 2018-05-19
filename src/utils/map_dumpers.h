#ifndef SLAM_CTOR_UTIL_MAP_DUMPERS
#define SLAM_CTOR_UTIL_MAP_DUMPERS

#include <cmath>
#include <fstream>
#include <string>

#include "../core/math_utils.h"
#include "../core/states/world.h"
#include "../core/states/state_data.h"
#include "../core/maps/grid_map.h"

template <typename GridMapType>
class GridMapToPgmDumber : public WorldMapObserver<GridMapType> {
  // Used PGM format discription: http://netpbm.sourceforge.net/doc/pgm.html
private:
  using IntensityType = unsigned char;
  static constexpr IntensityType Max_Intensity = 255;
private:
  class PgmHeader {
  public:
    PgmHeader(int w, int h, unsigned max_v)
      : _width{w}, _height{h}, _max_val{max_v} {}

    void write(std::ostream &os) const {
      os.write(_magic, sizeof(_magic));
      write_whitespace(os);
      write_integral(os, _width);
      write_whitespace(os);
      write_integral(os, _height);
      write_whitespace(os);
      write_integral(os, _max_val);
    }
  private:

    void write_whitespace(std::ostream &os) const {
      char new_line = '\n';
      os.write(&new_line, sizeof(new_line));
    }

    template <typename T>
    void write_integral(std::ostream &os, T value) const {
      auto str = std::to_string(value);
      os.write(str.c_str(), str.size());
    }

  private:
    char _magic[2] = {'P', '5'};
    int _width, _height;
    unsigned _max_val;
  };

public:
  GridMapToPgmDumber(const std::string &base_fname)
    : _base_fname{base_fname + "_"}
    , _id{0} {}

  void on_map_update(const GridMapType &map) override {
    std::ofstream dst(_base_fname + std::to_string(_id) + ".pgm",
                      std::ios::binary | std::ios::out);
    dump_map(dst, map);
    dst.close();
    ++_id;
  }

  static void dump_map(std::ofstream &os, const GridMapType &map) {
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
        auto occ = map.occupancy(area_id);
        auto value = 1.0 - (occ == -1 ? 0.5 : bound_value(0.0, occ, 1.0));
        auto intensity = static_cast<IntensityType>(Max_Intensity * value);
        static_assert(sizeof(intensity) == 1, "PGM insensity is not char");
        os.write(reinterpret_cast<char*>(&intensity), sizeof(intensity));
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
