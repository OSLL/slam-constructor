#include <gtest/gtest.h>

#include <memory>
#include <ostream>

#include "../mock_grid_cell.h"

#include "../../../src/core/maps/plain_grid_map.h"

class UnboundedPlainGridMapTest : public ::testing::Test {
protected: // methods
  UnboundedPlainGridMapTest()
    : map{std::make_shared<MockGridCell>(), {1, 1, 1}}
    , data{true, {0.5, 0.5}, {-1, -1}, 0} {}
protected: // fields
  UnboundedPlainGridMap map;
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

TEST_F(UnboundedPlainGridMapTest, expandRight) {
  map.update({2, 0}, data);
  ASSERT_EQ(MapInfo(map), MapInfo(3, 1, 0, 0));
}

TEST_F(UnboundedPlainGridMapTest, expandTop) {
  map.update({0, 2}, data);
  ASSERT_EQ(MapInfo(map), MapInfo(1, 3, 0, 0));
}

TEST_F(UnboundedPlainGridMapTest, expandLeft) {
  map.update({-2, 0}, data);
  ASSERT_EQ(MapInfo(map), MapInfo(3, 1, 2, 0));
}

TEST_F(UnboundedPlainGridMapTest, expandDown) {
  map.update({0, -2}, data);
  ASSERT_EQ(MapInfo(map), MapInfo(1, 3, 0, 2));
}

TEST_F(UnboundedPlainGridMapTest, expandRightTop) {
  map.update({2, 2}, data);
  ASSERT_EQ(MapInfo(map), MapInfo(3, 3, 0, 0));
}

TEST_F(UnboundedPlainGridMapTest, expandRightDown) {
  map.update({2, -2}, data);
  ASSERT_EQ(MapInfo(map), MapInfo(3, 3, 0, 2));
}

TEST_F(UnboundedPlainGridMapTest, expandLeftTop) {
  map.update({-2, 2}, data);
  ASSERT_EQ(MapInfo(map), MapInfo(3, 3, 2, 0));
}

TEST_F(UnboundedPlainGridMapTest, expandLeftDown) {
  map.update({-2, -2}, data);
  ASSERT_EQ(MapInfo(map), MapInfo(3, 3, 2, 2));
}

TEST_F(UnboundedPlainGridMapTest, noExpand) {
  map.update({0, 0}, data);
  ASSERT_EQ(MapInfo(map), MapInfo(1, 1, 0, 0));
}

TEST_F(UnboundedPlainGridMapTest, valueStorage) {
  static constexpr int Lim = 50;
  for (int i = -Lim; i != Lim; ++i) {
    for (int j = -Lim; j != Lim; ++j) {
      map.update({i, j}, {true, {(double)Lim * i + j, 0}, {0, 0}, 0});
    }
  }
  for (int i = -Lim; i != Lim; ++i) {
    for (int j = -Lim; j != Lim; ++j) {
      ASSERT_EQ((map[{i, j}]), ((double)Lim * i + j));
    }
  }

}

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
