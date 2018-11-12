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

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
