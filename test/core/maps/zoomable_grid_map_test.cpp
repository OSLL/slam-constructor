#include <gtest/gtest.h>

#include <memory>
#include <cmath>

#include "../../../src/core/maps/zoomable_grid_map.h"
#include "../../../src/core/maps/plain_grid_map.h"
#include "../../../src/core/maps/lazy_tiled_grid_map.h"

constexpr double Default_Occupancy_Prob = -42;

class TestGridCell : public GridCell {
public:
  TestGridCell() : GridCell{Occupancy{Default_Occupancy_Prob, 0}} {}
  std::unique_ptr<GridCell> clone() const override {
    return std::make_unique<TestGridCell>(*this);
  }
};

class ZoomableGridMapTest : public ::testing::Test {
protected: // methods
  ZoomableGridMapTest()
    : cell_proto{std::make_shared<TestGridCell>()}
    , map{cell_proto, {16, 16, 1}} {}
protected: // fields
  std::shared_ptr<TestGridCell> cell_proto;
  ZoomableGridMap<UnboundedPlainGridMap> map;
};

TEST_F(ZoomableGridMapTest, initAutozoomPowerOf2MapSize) {
  auto map = ZoomableGridMap<UnboundedPlainGridMap>{cell_proto, {16, 16, 1}};
  ASSERT_EQ(map.zoom_levels_nm(), 5);
}

TEST_F(ZoomableGridMapTest, initAutozoomSquareMap) {
  auto map = ZoomableGridMap<UnboundedPlainGridMap>{cell_proto, {17, 17, 1}};
  ASSERT_EQ(map.zoom_levels_nm(), 6);
}

TEST_F(ZoomableGridMapTest, initAutozoomWidthyMap) {
  auto map = ZoomableGridMap<UnboundedPlainGridMap>{cell_proto, {13, 8, 1}};
  ASSERT_EQ(map.zoom_levels_nm(), 5);
}

TEST_F(ZoomableGridMapTest, initAutozoomHeightyMap) {
  auto map = ZoomableGridMap<UnboundedPlainGridMap>{cell_proto, {3, 15, 1}};
  ASSERT_EQ(map.zoom_levels_nm(), 5);
}

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
  map.set_zoom_level(ZoomableGridMap<T>::Unzoomed_Map_Level);
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

template<typename T, int Width, int Height>
void test_read_write_inside_map(std::shared_ptr<GridCell> cell_proto) {
  auto map = ZoomableGridMap<T>{cell_proto, {Width, Height, 1}};
  smoke_map_init(map);

  // read values
  int scale = 1;
  for (unsigned lvl = 0; lvl < map.zoom_levels_nm(); ++lvl) {
    map.set_zoom_level(lvl);
    auto origin = map.origin();

    //std::cout << "Lvl: " << lvl << " Orig: " << map.origin();
    //std::cout << " W: " << map.width() << " H: " << map.height() << std::endl;

    auto raw_coord = DiscretePoint2D{0, 0};
    for (raw_coord.x = 0; raw_coord.x < map.width(); ++raw_coord.x) {
      for (raw_coord.y = 0; raw_coord.y < map.height(); ++raw_coord.y) {
        auto coord = raw_coord - origin;

        map.set_zoom_level(decltype(map)::Unzoomed_Map_Level);
        // prevent proxies creation
        const GridMap &read_access_map = map;
        double expected = 0;
        auto uzd_coord = DiscretePoint2D{scale * coord.x,
                                         scale * coord.y};
        auto uzd_delta = DiscretePoint2D{0, 0};
        for (uzd_delta.x = 0; uzd_delta.x < scale; ++uzd_delta.x) {
          for (uzd_delta.y = 0; uzd_delta.y < scale; ++uzd_delta.y) {
            expected = std::max(double(read_access_map[uzd_coord + uzd_delta]),
                                expected);
          }
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

/* FIXME: incorrect. Fix tests after zooming strategy implementation.
TEST_F(ZoomableGridMapTest, readWriteUPGM_1000x1000) {
  test_read_write_inside_map<UnboundedPlainGridMap, 1000, 1000>(cell_proto);
}

TEST_F(ZoomableGridMapTest, readWriteULTGM_1000x1000) {
  test_read_write_inside_map<UnboundedLazyTiledGridMap, 1000, 1000>(cell_proto);
}
*/

/* FIXME
TEST_F(ZoomableGridMapTest, zoomLevelExtension) {
  auto map = ZoomableGridMap<UnboundedPlainGridMap>{cell_proto, {16, 16, 1}};
  ASSERT_EQ(map.zoom_levels_nm(), 5);
  // init map
  map.set_zoom_level(decltype(map)::Unzoomed_Map_Level);
  auto val = AreaOccupancyObservation{true, Occupancy{128, 0},
                                      Point2D{0, 0}, 1};
  map[map.world_to_cell(23, 23)] += val;
  ASSERT_EQ(map.zoom_levels_nm(), 6);

  map[map.world_to_cell(24, 24)] += val;
  ASSERT_EQ(map.zoom_levels_nm(), 7);
}
*/

TEST_F(ZoomableGridMapTest, multipleProxyAccessTesting) {
  auto map = ZoomableGridMap<UnboundedPlainGridMap>{cell_proto, {16, 16, 1}};
  smoke_map_init(map);

  map.set_zoom_level(decltype(map)::Unzoomed_Map_Level);
  auto crd = DiscretePoint2D{-3, -3}; // map[coord] = 85
  map[crd] += AreaOccupancyObservation{true, Occupancy{map[crd] * map[crd], 0},
                                       Point2D{0, 0}, 1};
  ASSERT_EQ(map[crd], 85*85);
}



int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
