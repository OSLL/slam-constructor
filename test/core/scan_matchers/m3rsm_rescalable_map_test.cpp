#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <cmath>

#include "../mock_grid_cell.h"

#include "../../../src/core/maps/grid_rasterization.h"
#include "../../../src/core/scan_matchers/m3rsm_engine.h"
#include "../../../src/core/maps/rescalable_caching_grid_map.h"
#include "../../../src/core/maps/plain_grid_map.h"
#include "../../../src/core/maps/lazy_tiled_grid_map.h"

constexpr double Default_Occupancy_Prob = -42.0;

//------------------------------------------------------------------------------

class M3RSMRescalableMapTest : public ::testing::Test {
protected:
  template <typename BackMapType = UnboundedPlainGridMap>
  using TesteeMapType = M3RSMRescalableGridMap<BackMapType>;
protected: // methods
  M3RSMRescalableMapTest()
    : cell_proto{std::make_shared<MockGridCell>(Default_Occupancy_Prob)} {}
protected:

  void smoke_map_init(GridMap &map, double from, double step) {
    auto val = AreaOccupancyObservation{true, Occupancy{from, 0},
                                        Point2D{0, 0}, 1};

    auto coord = DiscretePoint2D{0, 0};
    for (coord.x = 0; coord.x < map.width(); ++coord.x) {
      for (coord.y = 0; coord.y < map.height(); ++coord.y) {
        map.update(map.internal2external(coord), val);
        assert(are_equal(map[map.internal2external(coord)],
                         val.occupancy.prob_occ));
        val.occupancy.prob_occ += step;
      }
    }
  }

  double estimate_impact(const GridCell &area) const {
    static const auto AOO = AreaOccupancyObservation{true, {1.0, 1.0},
                                                     {0, 0}, 1.0};
    return 1.0 - area.discrepancy(AOO);
  }

  double estimate_max_impact(const GridMap &map, const Rectangle &area) const {
    double expected = std::numeric_limits<double>::lowest();
    for (auto &a_id : GridRasterizedRectangle{map, area, false}.to_vector()) {
      if (map[a_id].is_unknown()) { continue; }
      expected = std::max(estimate_impact(map[a_id]), expected);
    }

    if (are_equal(expected, std::numeric_limits<double>::lowest())) {
      expected = Default_Occupancy_Prob;
    }
    return expected;
  }

  template<typename BaseMapType>
  void verify_map_state(TesteeMapType<BaseMapType> &map) {
    for (unsigned scale_id = map.finest_scale_id();
         scale_id <= map.coarsest_scale_id(); ++scale_id) {
      map.set_scale_id(scale_id);
      auto raw_crd = DiscretePoint2D{0, 0};
      for (raw_crd.x = 0; raw_crd.x < map.width(); ++raw_crd.x) {
        for (raw_crd.y = 0; raw_crd.y < map.height(); ++raw_crd.y) {
          auto coord = map.internal2external(raw_crd);
          auto area = map.world_cell_bounds(coord);
          map.set_scale_id(map.finest_scale_id());
          double expected = estimate_max_impact(map, area);
          map.set_scale_id(scale_id);
          ASSERT_EQ(expected, estimate_impact(map[coord]));
        }
      }
    }
  }

  template<typename T, int Width, int Height>
  void test_read_write_inside_map() {
    auto map = TesteeMapType<T>{cell_proto, {Width, Height, 1}};
    map.set_scale_id(map.finest_scale_id());
    smoke_map_init(map, 0, 1);
    verify_map_state<T>(map);
  }

protected: // fields
  std::shared_ptr<GridCell> cell_proto;
};

//------------------------------------------------------------------------------
// * Estimated scales number estimation on init
// * Approximated Maps access
// * Map reading writing
// Tested by RescalableCachingGridMapTest

//------------------------------------------------------------------------------
// Coarse level auto-update according to M3RSM policy (Max is propagated)

TEST_F(M3RSMRescalableMapTest, readWrite_16x16) {
  test_read_write_inside_map<UnboundedPlainGridMap, 16, 16>();
}

TEST_F(M3RSMRescalableMapTest, readWrite_16x1) {
  test_read_write_inside_map<UnboundedPlainGridMap, 16, 1>();
}

TEST_F(M3RSMRescalableMapTest, readWrite_1x16) {
  test_read_write_inside_map<UnboundedPlainGridMap, 1, 16>();
}

TEST_F(M3RSMRescalableMapTest, readWrite_15x15) {
  test_read_write_inside_map<UnboundedPlainGridMap, 15, 15>();
}

TEST_F(M3RSMRescalableMapTest, readWrite_33x33) {
  test_read_write_inside_map<UnboundedPlainGridMap, 33, 33>();
}

TEST_F(M3RSMRescalableMapTest, readWrite_15x3) {
  test_read_write_inside_map<UnboundedPlainGridMap, 15, 3>();
}

TEST_F(M3RSMRescalableMapTest, readWrite_3x15) {
  test_read_write_inside_map<UnboundedPlainGridMap, 3, 15>();
}

TEST_F(M3RSMRescalableMapTest, readWriteUPGM_1000x1000) {
   test_read_write_inside_map<UnboundedPlainGridMap, 1000, 1000>();
}

TEST_F(M3RSMRescalableMapTest, readWriteULTGM_1000x1000) {
  test_read_write_inside_map<UnboundedLazyTiledGridMap, 1000, 1000>();
}

TEST_F(M3RSMRescalableMapTest, unoccupiedPropagation) {
  // Tests bottom-up values propagation through default (unknown)
  // ones in case their values _below_ default
  auto const DFLT_OCC_PROB = 0.5, UNOCC_PROB = 0.0;
  cell_proto = std::make_shared<MockGridCell>(DFLT_OCC_PROB);
  auto map = TesteeMapType<>{cell_proto, GridMapParams{16, 16, 1}};
  smoke_map_init(map, UNOCC_PROB, 0);
  verify_map_state<typename decltype(map)::BackMapT>(map);
}

TEST_F(M3RSMRescalableMapTest, approximationLevelExtension) {
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
