#include <gtest/gtest.h>

#include <memory>

#include "../mock_grid_cell.h"

#include "../../../src/core/maps/lazy_tiled_grid_map.h"

class UnboundedLazyTiledGridMapTest : public ::testing::Test {
protected: // methods
  UnboundedLazyTiledGridMapTest()
    : map{std::make_shared<MockGridCell>(), {1, 1, 1}}
    , data{true, {0.5, 0.5}, {-1, -1}, 0} {}
protected: // fields
  UnboundedLazyTiledGridMap map;
  AreaOccupancyObservation data;
};

struct MapInfo {
// methods
  MapInfo(int w_, int h_, int o_x, int o_y) : w{w_}, h{h_}, origin{o_x, o_y} {}
  MapInfo(const GridMap& map) :
    w{map.width()}, h{map.height()}, origin{map.origin()} {}

  bool operator==(const MapInfo& that) const {
    return (w == that.w) && (h == that.h) && (origin == that.origin);
  }
// fields
  int w, h;
  DiscretePoint2D origin;
};

std::ostream &operator<<(std::ostream &stream, const MapInfo &mi) {
  stream << "w: " << mi.w << ", h: " << mi.h;
  return stream << ", origin: ("<< mi.origin.x << ", " << mi.origin.y << ")";
}

TEST_F(UnboundedLazyTiledGridMapTest, readMapCopy) {
  static constexpr int Lim = 50;
  for (int i = -Lim; i != Lim; ++i) {
    for (int j = -Lim; j != Lim; ++j) {
      map.update({i, j}, {true, {(double)Lim * i + j, 0}, {0, 0}, 0});
    }
  }
  UnboundedLazyTiledGridMap map_copy = map;

  for (int i = -Lim; i != Lim; ++i) {
    for (int j = -Lim; j != Lim; ++j) {
      ASSERT_EQ((map[{i, j}]), ((double)Lim * i + j));
      ASSERT_EQ((map_copy[{i, j}]), ((double)Lim * i + j));
    }
  }

}

TEST_F(UnboundedLazyTiledGridMapTest, modifyMapCopy) {
  static constexpr int Lim = 50;
  for (int i = -Lim; i != Lim; ++i) {
    for (int j = -Lim; j != Lim; ++j) {
      map.update({i, j}, {true, {(double)Lim * i + j, 0}, {0, 0}, 0});
    }
  }
  UnboundedLazyTiledGridMap map_copy = map;
  for (int i = -Lim; i != Lim; ++i) {
    for (int j = -Lim; j != Lim; ++j) {
      map_copy.update({i, j}, {true, {(double)2*Lim * i + j, 0}, {0, 0}, 0});
    }
  }

  for (int i = -Lim; i != Lim; ++i) {
    for (int j = -Lim; j != Lim; ++j) {
      ASSERT_EQ((map[{i, j}]), ((double)Lim * i + j));
      ASSERT_EQ((map_copy[{i, j}]), ((double)2*Lim * i + j));
    }
  }

}


int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
