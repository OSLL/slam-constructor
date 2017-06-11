#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <cmath>

#include "../mock_grid_cell.h"

#include "../../../src/core/maps/grid_rasterization.h"
#include "../../../src/core/maps/rescalable_caching_grid_map.h"
#include "../../../src/core/maps/plain_grid_map.h"
#include "../../../src/core/maps/lazy_tiled_grid_map.h"

constexpr double Default_Occupancy_Prob = -42.0;

//------------------------------------------------------------------------------

class RescalableCachingGridMapTest : public ::testing::Test {
protected:
  template <typename BackMapType = UnboundedPlainGridMap>
  using TesteeMapType = RescalableCachingGridMap<BackMapType>;
protected: // methods
  RescalableCachingGridMapTest()
    : cell_proto{std::make_shared<MockGridCell>(Default_Occupancy_Prob)} {}
protected:

  template <int W, int H, int ExpectedCoarsestScaleId>
  void test_max_scale_id_estimation() const {
    auto map = TesteeMapType<UnboundedPlainGridMap>{cell_proto, {W, H, 1}};
    ASSERT_EQ(ExpectedCoarsestScaleId, map.coarsest_scale_id());
  }

  void smoke_map_init(GridMap &map) {
    auto val = AreaOccupancyObservation{true, Occupancy{0, 0},
                                        Point2D{0, 0}, 1};

    auto coord = DiscretePoint2D{0, 0};
    for (coord.x = 0; coord.x < map.width(); ++coord.x) {
      for (coord.y = 0; coord.y < map.height(); ++coord.y) {
        map.update(map.internal2external(coord), val);
        assert(are_equal(map[map.internal2external(coord)],
                         val.occupancy.prob_occ));
        val.occupancy.prob_occ += 1;
      }
    }
  }

  double estimate_max_value(const GridMap &map, const Rectangle &area) const {
    double expected = std::numeric_limits<double>::lowest();
    for (auto &coord : GridRasterizedRectangle{map, area, false}.to_vector()) {
      expected = std::max(double(map[coord]), expected);
    }

    return expected;
  }

  template<typename T, int Width, int Height>
  void test_read_write_inside_map() {
    auto map = TesteeMapType<T>{cell_proto, {Width, Height, 1}};
    map.set_scale_id(map.finest_scale_id());
    smoke_map_init(map);
    // read values
    int scale = 1;
    for (unsigned scale_id = map.finest_scale_id();
         scale_id <= map.coarsest_scale_id(); ++scale_id) {
      map.set_scale_id(scale_id);
      auto raw_crd = DiscretePoint2D{0, 0};
      for (raw_crd.x = 0; raw_crd.x < map.width(); ++raw_crd.x) {
        for (raw_crd.y = 0; raw_crd.y < map.height(); ++raw_crd.y) {
          auto coord = map.internal2external(raw_crd);
          auto area = map.world_cell_bounds(coord);
          map.set_scale_id(map.finest_scale_id());
          double expected = estimate_max_value(map, area);
          map.set_scale_id(scale_id);
          ASSERT_EQ(expected, double(map[coord]));
        }
      }
      scale *= map.Map_Scale_Factor;;
    }
  }

protected: // fields
  std::shared_ptr<GridCell> cell_proto;
};

//------------------------------------------------------------------------------
// Estimated scales number estimation on init

TEST_F(RescalableCachingGridMapTest, initPowerOf2MapSize) {
  test_max_scale_id_estimation<16, 16, 4>();
}

TEST_F(RescalableCachingGridMapTest, initSquareMap) {
  test_max_scale_id_estimation<17, 17, 5>();
}

TEST_F(RescalableCachingGridMapTest, initWidthyMap) {
  test_max_scale_id_estimation<13, 8, 4>();
}

TEST_F(RescalableCachingGridMapTest, initHeightyMap) {
  test_max_scale_id_estimation<3, 15, 4>();
}

//------------------------------------------------------------------------------
// Approximated Maps access

TEST_F(RescalableCachingGridMapTest, dimensionsManualScaleSwitching) {
  unsigned prev_w = 32, prev_h = 32;
  auto map = TesteeMapType<>{cell_proto, {16, 16, 1}};

  for (unsigned scale_id = map.finest_scale_id();
       scale_id <= map.coarsest_scale_id(); ++scale_id) {
    map.set_scale_id(scale_id);
    ASSERT_LT(map.width(), prev_w);
    ASSERT_LT(map.height(), prev_h);
    prev_w = map.width();
    prev_h = map.height();
  }
  ASSERT_EQ(prev_w, 1);
  ASSERT_EQ(prev_h, 1);
}

TEST_F(RescalableCachingGridMapTest, theCoarsestMapSmokeCheck) {
  auto map = TesteeMapType<>{cell_proto, {16, 16, 1}};
  map.set_scale_id(map.coarsest_scale_id());

  ASSERT_EQ(DiscretePoint2D(0, 0), map.world_to_cell(100000, 100000));
  ASSERT_EQ(DiscretePoint2D(0, 0), map.world_to_cell(-100000, 100000));
  ASSERT_EQ(DiscretePoint2D(0, 0), map.world_to_cell(-100000, -100000));
  ASSERT_EQ(DiscretePoint2D(0, 0), map.world_to_cell(100000, -100000));
}


//------------------------------------------------------------------------------
// Map reading writing

TEST_F(RescalableCachingGridMapTest, initDefaultValues) {
  auto map = TesteeMapType<>{cell_proto, {3, 15, 1}};
  for (unsigned scale_id = map.finest_scale_id();
       scale_id <= map.coarsest_scale_id(); ++scale_id) {
    map.set_scale_id(scale_id);
    auto coord = DiscretePoint2D{0, 0};
    for (; coord.x < map.width(); ++coord.x) {
      for (; coord.y < map.height(); ++coord.y) {
        ASSERT_EQ(double(map[map.internal2external(coord)]),
                  Default_Occupancy_Prob);
      }
    }
  }
}

//------------------------------------------------------------------------------
// Approximations update

TEST_F(RescalableCachingGridMapTest, readWrite_16x16) {
  test_read_write_inside_map<UnboundedPlainGridMap, 16, 16>();
}

TEST_F(RescalableCachingGridMapTest, readWrite_16x1) {
  test_read_write_inside_map<UnboundedPlainGridMap, 16, 1>();
}

TEST_F(RescalableCachingGridMapTest, readWrite_1x16) {
  test_read_write_inside_map<UnboundedPlainGridMap, 1, 16>();
}

TEST_F(RescalableCachingGridMapTest, readWrite_15x15) {
  test_read_write_inside_map<UnboundedPlainGridMap, 15, 15>();
}

TEST_F(RescalableCachingGridMapTest, readWrite_33x33) {
  test_read_write_inside_map<UnboundedPlainGridMap, 33, 33>();
}

TEST_F(RescalableCachingGridMapTest, readWrite_15x3) {
  test_read_write_inside_map<UnboundedPlainGridMap, 15, 3>();
}

TEST_F(RescalableCachingGridMapTest, readWrite_3x15) {
  test_read_write_inside_map<UnboundedPlainGridMap, 3, 15>();
}

TEST_F(RescalableCachingGridMapTest, readWriteUPGM_1000x1000) {
   test_read_write_inside_map<UnboundedPlainGridMap, 1000, 1000>();
}

TEST_F(RescalableCachingGridMapTest, readWriteULTGM_1000x1000) {
  test_read_write_inside_map<UnboundedLazyTiledGridMap, 1000, 1000>();
}

TEST_F(RescalableCachingGridMapTest, approximationLevelExtension) {
  auto map = TesteeMapType<>{cell_proto, {16, 16, 1}};
  map.set_scale_id(map.finest_scale_id());
  ASSERT_EQ(map.coarsest_scale_id(), 4);

  // init map
  auto val = AreaOccupancyObservation{true, Occupancy{128, 0},
                                      Point2D{0, 0}, 1};
  {
    auto updated_point = Point2D{15, 15};
    map.update(map.world_to_cell(updated_point), val);
    ASSERT_EQ(map.coarsest_scale_id(), 5);
  }

  {
    auto updated_point = Point2D{16, 16};
    map.update(map.world_to_cell(updated_point), val);
    ASSERT_EQ(map.coarsest_scale_id(), 6);
  }
}

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
