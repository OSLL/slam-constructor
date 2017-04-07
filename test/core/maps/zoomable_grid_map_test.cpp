#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <cmath>

#include "../mock_grid_cell.h"

#include "../../../src/core/maps/zoomable_grid_map.h"
#include "../../../src/core/maps/plain_grid_map.h"
#include "../../../src/core/maps/lazy_tiled_grid_map.h"

constexpr double Default_Occupancy_Prob = -42.0;

class ZoomableGridMapTest : public ::testing::Test {
protected: // methods
  ZoomableGridMapTest()
    : cell_proto{std::make_shared<MockGridCell>(Default_Occupancy_Prob)} {}
protected: // fields
  std::shared_ptr<GridCell> cell_proto;
};

//------------------------------------------------------------------------------
// Zoom Levels estimation on init

TEST_F(ZoomableGridMapTest, initAutozoomPowerOf2MapSize) {
  auto map = ZoomableGridMap<UnboundedPlainGridMap>{cell_proto, {16, 16, 1}};
  ASSERT_EQ(5, map.zoom_levels_nm());
}

TEST_F(ZoomableGridMapTest, initAutozoomSquareMap) {
  auto map = ZoomableGridMap<UnboundedPlainGridMap>{cell_proto, {17, 17, 1}};
  ASSERT_EQ(6, map.zoom_levels_nm());
}

TEST_F(ZoomableGridMapTest, initAutozoomWidthyMap) {
  auto map = ZoomableGridMap<UnboundedPlainGridMap>{cell_proto, {13, 8, 1}};
  ASSERT_EQ(5, map.zoom_levels_nm());
}

TEST_F(ZoomableGridMapTest, initAutozoomHeightyMap) {
  auto map = ZoomableGridMap<UnboundedPlainGridMap>{cell_proto, {3, 15, 1}};
  ASSERT_EQ(5, map.zoom_levels_nm());
}

//------------------------------------------------------------------------------
// Map level switching

TEST_F(ZoomableGridMapTest, dimensionsSwitchingWithTheLevel) {
  unsigned prev_w = 32, prev_h = 32;
  auto map = ZoomableGridMap<UnboundedPlainGridMap>{cell_proto, {16, 16, 1}};

  for (unsigned lvl = 0; lvl < map.zoom_levels_nm(); ++lvl) {
    map.set_zoom_level(lvl);
    ASSERT_LT(map.width(), prev_w);
    ASSERT_LT(map.height(), prev_h);
    prev_w = map.width();
    prev_h = map.height();
  }
  ASSERT_EQ(prev_w, 1);
  ASSERT_EQ(prev_h, 1);
}

//------------------------------------------------------------------------------
// The coarsest map

TEST_F(ZoomableGridMapTest, smokeTheCoarsestMapCheck) {
  auto map = ZoomableGridMap<UnboundedPlainGridMap>{cell_proto, {16, 16, 1}};
  map.set_zoom_level(map.coarsest_zoom_level());
  ASSERT_EQ(DiscretePoint2D(0, 0), map.world_to_cell(100000, 100000));
  ASSERT_EQ(DiscretePoint2D(0, 0), map.world_to_cell(-100000, 100000));
  ASSERT_EQ(DiscretePoint2D(0, 0), map.world_to_cell(-100000, -100000));
  ASSERT_EQ(DiscretePoint2D(0, 0), map.world_to_cell(100000, -100000));
}

//------------------------------------------------------------------------------
// Map reading writing

TEST_F(ZoomableGridMapTest, initDefaultValuesConstRead) {
  auto map = ZoomableGridMap<UnboundedPlainGridMap>{cell_proto, {3, 15, 1}};
  for (unsigned lvl = 0; lvl < map.zoom_levels_nm(); ++lvl) {
    map.set_zoom_level(lvl);
    auto coord = DiscretePoint2D{0, 0};
    auto origin = map.origin();
    const ZoomableGridMap<UnboundedPlainGridMap> &const_map = map;
    for (; coord.x < const_map.width(); ++coord.x) {
      for (; coord.y < const_map.height(); ++coord.y) {
        ASSERT_EQ(double(const_map[coord - origin]), Default_Occupancy_Prob);
      }
    }
  }
}

TEST_F(ZoomableGridMapTest, initDefaultValuesNonConstRead) {
  auto map = ZoomableGridMap<UnboundedPlainGridMap>{cell_proto, {3, 15, 1}};
  for (unsigned lvl = 0; lvl < map.zoom_levels_nm(); ++lvl) {
    map.set_zoom_level(lvl);

    auto coord = DiscretePoint2D{0, 0};
    auto origin = map.origin();
    for (; coord.x < map.width(); ++coord.x) {
      for (; coord.y < map.height(); ++coord.y) {
        ASSERT_EQ(double(map[coord - origin]), Default_Occupancy_Prob);
      }
    }
  }
}

template <typename T>
void smoke_map_init(ZoomableGridMap<T> &map) {
  // init map
  map.set_zoom_level(ZoomableGridMap<T>::finest_zoom_level());
  auto val = AreaOccupancyObservation{true, Occupancy{0, 0}, Point2D{0, 0}, 1};
  auto origin = map.origin();
  auto coord = DiscretePoint2D{0, 0};
  for (coord.x = 0; coord.x < map.width(); ++coord.x) {
    for (coord.y = 0; coord.y < map.height(); ++coord.y) {
      map[coord - origin] += val;
      val.occupancy.prob_occ += 1;
    }
  }
}

double estimate_max_value(const GridMap &map, const Rectangle &bnds) {
  double expected = std::numeric_limits<double>::lowest();
  auto bot_left = map.world_to_cell(bnds.left(), bnds.bot());
  auto top_right = map.world_to_cell(bnds.right(), bnds.top());
  auto coord = DiscretePoint2D{0, 0};
  for (coord.x = bot_left.x; coord.x < top_right.x; ++coord.x) {
    for (coord.y = bot_left.y; coord.y < top_right.y; ++coord.y) {
      expected = std::max(double(map[coord]), expected);
    }
  }
  return expected;
}

double estimate_max_value(const GridMap &map) {
  double expected = 0;
  auto coord = DiscretePoint2D{0, 0};
  for (coord.x = 0; coord.x < map.width(); ++coord.x) {
    for (coord.y = 0; coord.y < map.height(); ++coord.y) {
      expected = std::max(double(map[coord]), expected);
    }
  }
  return expected;
}

template<typename T, int Width, int Height>
void test_read_write_inside_map(std::shared_ptr<GridCell> cell_proto) {
  auto map = ZoomableGridMap<T>{cell_proto, {Width, Height, 1}};
  smoke_map_init(map);

  // read values
  int scale = 1;
  for (unsigned lvl = 0; lvl < map.zoom_levels_nm(); ++lvl) {
    map.set_zoom_level(lvl);
    auto origin = map.origin();

    auto raw_coord = DiscretePoint2D{0, 0};
    for (raw_coord.x = 0; raw_coord.x < map.width(); ++raw_coord.x) {
      for (raw_coord.y = 0; raw_coord.y < map.height(); ++raw_coord.y) {
        auto coord = raw_coord - origin;
        double expected = std::numeric_limits<double>::lowest();
        if (lvl != map.zoom_levels_nm() - 1) {
          auto bnds = map.world_cell_bounds(coord);
          map.set_zoom_level(decltype(map)::finest_zoom_level());
          expected = estimate_max_value(map, bnds);
        } else {
          map.set_zoom_level(decltype(map)::finest_zoom_level());
          expected = estimate_max_value(map);
        }

        map.set_zoom_level(lvl);
        ASSERT_EQ(expected, double(map[coord]));
      }
    }
    scale *= 2;
  }
}

//TODO: think about "recursive" macro calls
TEST_F(ZoomableGridMapTest, readWrite_16x16) {
  test_read_write_inside_map<UnboundedPlainGridMap, 16, 16>(cell_proto);
}

TEST_F(ZoomableGridMapTest, readWrite_16x1) {
  test_read_write_inside_map<UnboundedPlainGridMap, 16, 1>(cell_proto);
}

TEST_F(ZoomableGridMapTest, readWrite_1x16) {
  test_read_write_inside_map<UnboundedPlainGridMap, 1, 16>(cell_proto);
}

TEST_F(ZoomableGridMapTest, readWrite_15x15) {
  test_read_write_inside_map<UnboundedPlainGridMap, 15, 15>(cell_proto);
}

TEST_F(ZoomableGridMapTest, readWrite_33x33) {
  test_read_write_inside_map<UnboundedPlainGridMap, 33, 33>(cell_proto);
}

TEST_F(ZoomableGridMapTest, readWrite_15x3) {
  test_read_write_inside_map<UnboundedPlainGridMap, 15, 3>(cell_proto);
}

TEST_F(ZoomableGridMapTest, readWrite_3x15) {
  test_read_write_inside_map<UnboundedPlainGridMap, 3, 15>(cell_proto);
}

TEST_F(ZoomableGridMapTest, readWriteUPGM_1000x1000) {
  test_read_write_inside_map<UnboundedPlainGridMap, 1000, 1000>(cell_proto);
}

TEST_F(ZoomableGridMapTest, readWriteULTGM_1000x1000) {
  test_read_write_inside_map<UnboundedLazyTiledGridMap, 1000, 1000>(cell_proto);
}

TEST_F(ZoomableGridMapTest, zoomLevelExtension) {
  auto map = ZoomableGridMap<UnboundedPlainGridMap>{cell_proto, {16, 16, 1}};
  ASSERT_EQ(map.zoom_levels_nm(), 5);
  // init map
  map.set_zoom_level(decltype(map)::finest_zoom_level());
  auto val = AreaOccupancyObservation{true, Occupancy{128, 0},
                                      Point2D{0, 0}, 1};
  map[map.world_to_cell(23, 23)] += val;
  ASSERT_EQ(map.zoom_levels_nm(), 6);

  map[map.world_to_cell(24, 24)] += val;
  ASSERT_EQ(map.zoom_levels_nm(), 7);
}

TEST_F(ZoomableGridMapTest, multipleProxyAccessTesting) {
  auto map = ZoomableGridMap<UnboundedPlainGridMap>{cell_proto, {16, 16, 1}};
  smoke_map_init(map);

  map.set_zoom_level(decltype(map)::finest_zoom_level());
  auto crd = DiscretePoint2D{-3, -3}; // map[coord] = 85
  map[crd] += AreaOccupancyObservation{true, Occupancy{map[crd] * map[crd], 0},
                                       Point2D{0, 0}, 1};
  ASSERT_EQ(map[crd], 85*85);
}

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
