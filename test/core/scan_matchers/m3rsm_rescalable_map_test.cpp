#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <cmath>

#include "../mock_grid_cell.h"

#include "../../../src/core/maps/grid_rasterization.h"
#include "../../../src/core/scan_matchers/m3rsm_engine.h"
#include "../../../src/core/scan_matchers/observation_impact_estimators.h"
#include "../../../src/core/maps/rescalable_caching_grid_map.h"
#include "../../../src/core/maps/plain_grid_map.h"
#include "../../../src/core/maps/lazy_tiled_grid_map.h"

constexpr double Default_Occupancy_Prob = -42.0;

//------------------------------------------------------------------------------

class M3RSMRescalableMapTest : public ::testing::Test {
protected:
  using DefaultBackMapType = UnboundedPlainGridMap;
  template <typename BackMapType = DefaultBackMapType>
  using TesteeMapType = M3RSMRescalableGridMap<BackMapType>;
  using OIE = DiscrepancyOIE; // TODO: templatize
protected: // methods
  M3RSMRescalableMapTest()
    : cell_proto{std::make_shared<MockGridCell>(Default_Occupancy_Prob)}
    , oie{std::make_shared<OIE>()} {}
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
    return OIE{}.estimate_obstacle_impact(area);
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

  template <typename BaseMapType>
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

  template <typename T, int Width, int Height>
  void test_read_write_inside_map() {
    auto map = make_map<T>(Width, Height, 1);
    map.set_scale_id(map.finest_scale_id());
    smoke_map_init(map, 0, 1);
    verify_map_state<T>(map);
  }

  template <typename T = DefaultBackMapType>
  TesteeMapType<T> make_map(int width, int height, double scale) {
    return TesteeMapType<T>{oie, cell_proto, {width, height, 1}};
  }

  template <typename T = DefaultBackMapType>
  void ensure_equal(TesteeMapType<T> &a, TesteeMapType<T> &b) {
    auto orig_scale_a = a.scale_id();
    auto orig_scale_b = b.scale_id();

    ASSERT_EQ(a.finest_scale_id(), b.finest_scale_id());
    ASSERT_EQ(a.coarsest_scale_id(), b.coarsest_scale_id());
    ASSERT_EQ(a.origin(), b.origin());
    for (auto id = a.finest_scale_id(); id <= a.coarsest_scale_id(); ++id) {
      a.set_scale_id(id);
      b.set_scale_id(id);

      auto raw_crd = DiscretePoint2D{0, 0};
      for (raw_crd.x = 0; raw_crd.x < a.width(); ++raw_crd.x) {
        for (raw_crd.y = 0; raw_crd.y < a.height(); ++raw_crd.y) {
          auto area_id_a = a.internal2external(raw_crd);
          auto area_id_b = b.internal2external(raw_crd);
          ASSERT_EQ(double(a[area_id_a]), double(b[area_id_b]));
        }
      }
    }

    a.set_scale_id(orig_scale_a);
    b.set_scale_id(orig_scale_b);
  }

protected: // fields
  std::shared_ptr<GridCell> cell_proto;
  std::shared_ptr<OIE> oie;
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
  auto map = make_map<>(16, 16, 1);
  smoke_map_init(map, UNOCC_PROB, 0);
  verify_map_state<typename decltype(map)::BackMapT>(map);
}

TEST_F(M3RSMRescalableMapTest, approximationLevelExtension) {
  auto map = make_map<>(16, 16, 1);
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

TEST_F(M3RSMRescalableMapTest, serialization) {
  auto original_map = make_map<UnboundedPlainGridMap>(100, 100, 1);
  original_map.set_scale_id(original_map.finest_scale_id());
  smoke_map_init(original_map, 0, 1);

  auto buf = original_map.save_state();
  auto restored_map = make_map<UnboundedPlainGridMap>(0, 0, 0);
  restored_map.load_state(buf);

  ensure_equal(original_map, restored_map);
}

TEST_F(M3RSMRescalableMapTest, noSlicingOnUpdate) {
  auto map = make_map<UnboundedPlainGridMap>(100, 100, 1);
  smoke_map_init(map, 0, 1);

  for (auto id = map.finest_scale_id(); id <= map.coarsest_scale_id(); ++id) {
    map.set_scale_id(id);
    auto raw_crd = DiscretePoint2D{0, 0};
    for (raw_crd.x = 0; raw_crd.x < map.width(); ++raw_crd.x) {
      for (raw_crd.y = 0; raw_crd.y < map.height(); ++raw_crd.y) {
        auto area_id = map.internal2external(raw_crd);
        auto area_ptr = dynamic_cast<const MockGridCell*>(&map[area_id]);
        ASSERT_NE(area_ptr, nullptr);
      }
    }
  }
}

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
