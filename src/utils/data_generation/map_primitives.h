#ifndef SLAM_CTOR_UTILS_DG_MAP_PRIMITIVES_INCLUDED
#define SLAM_CTOR_UTILS_DG_MAP_PRIMITIVES_INCLUDED

#include <string>
#include <iostream>
#include <sstream>
#include <utility>
#include <vector>

#include "../../core/geometry_utils.h"

class MapPrimitive {
public:
  static constexpr int Unknown_Value = -1;
public: // methods
  virtual int width() const { return Unknown_Value; };
  virtual int height() const { return Unknown_Value; };
  // any rectangle guaranteed to be free; origin - top-left
  virtual std::vector<Rectangle> free_space() const {
    return {};
  };
  virtual std::istream& to_stream() const = 0;
};

class TextRasterMapPrimitive : public MapPrimitive {
public: // consts
  static constexpr char Completely_Occupied_Marker = '+';
  static constexpr char Completely_Free_Marker = ' ';
  static constexpr char Row_End_Marker = '\n';
public: // methods
  std::istream& to_stream() const override {
    return _stream_pin = std::stringstream{text_raster()};
  }
protected: // methods
  virtual std::string text_raster() const = 0;
private:
  mutable std::stringstream _stream_pin;
};

// TODO: move to *.cpp
constexpr char TextRasterMapPrimitive::Completely_Occupied_Marker;
constexpr char TextRasterMapPrimitive::Completely_Free_Marker;
constexpr char TextRasterMapPrimitive::Row_End_Marker;

//------------------------------------------------------------------------------
// Actual Text Raster Primitives
//
// Responsibilities:
//   * provides a stream with text raster
//   * knows actual geometry of the primitive (opt)
//   * generates the primitive (opt)


/*

 <-- w -->
 +++++++++ ^              <-- BoundPosition::Top
 +       + |
 +       + h
 +       + |
 +       + v

 */
class CecumTextRasterMapPrimitive : public TextRasterMapPrimitive {
public: // const
  enum class BoundPosition { Left, Right, Top, Bot};
public: // methods
  CecumTextRasterMapPrimitive(int w, int h,
                              BoundPosition bnd_pos = BoundPosition::Top)
    : _width{w}, _height{h}, _bnd_pos{bnd_pos} {

    assert(0 < _width && 0 < _height);
    bool is_bnd_horiz = _bnd_pos == BoundPosition::Top ||
                        _bnd_pos == BoundPosition::Bot;
    if (is_bnd_horiz) {
      _raw_cecum = generate_horizontally_bounded_cecum(_bnd_pos);
    } else {
      _raw_cecum = generate_vertically_bounded_cecum(_bnd_pos);
    }
  }

  int width() const override { return _width; };
  int height() const override { return _height; };
  std::vector<Rectangle> free_space() const override {
    if (height() <= 2 || width() <= 2) {
      return {Rectangle{0, 0, 0, 0}};
    }
    switch (_bnd_pos) {
    case BoundPosition::Top:
      return {Rectangle(-height() + 1, 0, 1, width() - 1)};
    case BoundPosition::Bot:
      return {Rectangle(-height() + 2, 1, 1, width() - 1)};
    case BoundPosition::Left:
      return {Rectangle(-height() + 2, 0, 1, width())};
    case BoundPosition::Right:
      return {Rectangle(-height() + 2, 0, 0, width() - 1)};
    }
    return TextRasterMapPrimitive::free_space();
  };

protected: // methods
  std::string text_raster() const override { return _raw_cecum; };
private: // methods
  std::string generate_horizontally_bounded_cecum(BoundPosition bnd_pos) {
    assert(bnd_pos == BoundPosition::Top || bnd_pos == BoundPosition::Bot);

    std::string cecum;
    for (int row_i = 0; row_i < height(); ++row_i) {
      if ((row_i == 0 && bnd_pos == BoundPosition::Top) ||
          (row_i == height() - 1 && bnd_pos == BoundPosition::Bot)) {
        // generate horizontal bound
        cecum += std::string(width(), Completely_Occupied_Marker);
        cecum += Row_End_Marker;
      } else {
        cecum += Completely_Occupied_Marker;
        for (int col_i = 1; col_i < width(); ++col_i) {
          cecum += col_i == width() - 1 ? Completely_Occupied_Marker
                                        : Completely_Free_Marker;
        }
        cecum += Row_End_Marker;
      }
    }
    return cecum;
  }

  std::string generate_vertically_bounded_cecum(BoundPosition bnd_pos) {
    assert(bnd_pos == BoundPosition::Left || bnd_pos == BoundPosition::Right);

    std::string cecum(width(), Completely_Occupied_Marker);
    cecum += Row_End_Marker;
    for (int row_i = 1; row_i < height(); ++row_i) {
      if (row_i == height() - 1) {
        cecum += std::string(width(), Completely_Occupied_Marker);
        continue;
      }

      for (int col_i = 0; col_i < width(); ++col_i) {
        bool is_occ = width() == 1 ||
          (col_i == 0 && bnd_pos == BoundPosition::Left) ||
          (col_i == width() - 1 && bnd_pos == BoundPosition::Right);

        cecum += is_occ ? Completely_Occupied_Marker : Completely_Free_Marker;
      }
      cecum += Row_End_Marker;
    }
    cecum += Row_End_Marker;
    return cecum;
  }

private: // fields
  int _width, _height;
  std::string _raw_cecum;
  BoundPosition _bnd_pos;
};

#endif
