#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <cmath>

#include "../mock_grid_cell.h"

#include "../../../src/core/maps/grid_approximator.h"
#include "../../../src/core/maps/plain_grid_map.h"
#include "../../../src/core/maps/lazy_tiled_grid_map.h"

constexpr double Default_Occupancy_Prob = -42.0;

//------------------------------------------------------------------------------

class Pow2CachedPlainMaxApproximatorTest : public ::testing::Test {
protected: // methods
  using Policy = PlainMaxApproximationPolicy;
  template <typename T>
  using Approx = PowNCachedOGMA<T, 2>;
  Pow2CachedPlainMaxApproximatorTest()
    : cell_proto{std::make_shared<MockGridCell>(Default_Occupancy_Prob)}
    , policy{std::make_shared<Policy>(Default_Occupancy_Prob)} {}
protected:

  template <typename T = UnboundedPlainGridMap>
  std::shared_ptr<Approx<T>> create_approximator(const GridMap &map) const {
    auto approx = std::make_shared<Approx<T>>(policy);
    OccupancyGridMapApproximator::watch_master_map(map, approx);
    return approx;
  }

  template <int W, int H, int ExpectedMaxLvl>
  void check_approximation_level_nm() const {
    auto map = UnboundedPlainGridMap{cell_proto, {W, H, 1}};
    auto approximator = create_approximator<UnboundedPlainGridMap>(map);
    ASSERT_EQ(ExpectedMaxLvl, approximator->max_approximation_level());
  }

  void smoke_map_init(GridMap &map) {
    auto val = AreaOccupancyObservation{true, Occupancy{0, 0},
                                        Point2D{0, 0}, 1};

    auto coord = DiscretePoint2D{0, 0};
    for (coord.x = 0; coord.x < map.width(); ++coord.x) {
      for (coord.y = 0; coord.y < map.height(); ++coord.y) {
        map.update(map.internal2external(coord), val);
        val.occupancy.prob_occ += 1;
      }
    }
  }

  double estimate_max_value(const GridMap &map, const GridMap::Coord &coord,
                            const GridMap &fine_map) {
    double expected = std::numeric_limits<double>::lowest();
    auto pnt = map.cell_to_world(coord);
    for (auto &coord : policy->project_point(map, fine_map, pnt)) {
      expected = std::max(double(fine_map[coord]), expected);
    }
    return expected;
  }

  template<typename T, int Width, int Height>
  void test_read_write_inside_map() {
    auto fine_map = T{cell_proto, {Width, Height, 1}};
    auto apprx = create_approximator<T>(fine_map);
    smoke_map_init(fine_map);

    // read values
    int scale = 1;
    for (unsigned lvl = 0; lvl <= apprx->max_approximation_level(); ++lvl) {
      auto &coarse_map = apprx->map(lvl);

      auto raw_crd = DiscretePoint2D{0, 0};
      for (raw_crd.x = 0; raw_crd.x < coarse_map.width(); ++raw_crd.x) {
        for (raw_crd.y = 0; raw_crd.y < coarse_map.height(); ++raw_crd.y) {
          auto coord = coarse_map.internal2external(raw_crd);
          double expected = estimate_max_value(coarse_map, coord, fine_map);
          ASSERT_EQ(expected, double(coarse_map[coord]));
        }
      }
      scale *= 2;
    }
  }

protected: // fields
  std::shared_ptr<GridCell> cell_proto;
  std::shared_ptr<ApproximationPolicy> policy;
};

//------------------------------------------------------------------------------
// approximation Levels estimation on init

TEST_F(Pow2CachedPlainMaxApproximatorTest, initAutozoomPowerOf2MapSize) {
  check_approximation_level_nm<16, 16, 4>();
}


TEST_F(Pow2CachedPlainMaxApproximatorTest, initAutozoomSquareMap) {
  check_approximation_level_nm<17, 17, 5>();
}

TEST_F(Pow2CachedPlainMaxApproximatorTest, initAutozoomWidthyMap) {
  check_approximation_level_nm<13, 8, 4>();
}

TEST_F(Pow2CachedPlainMaxApproximatorTest, initAutozoomHeightyMap) {
  check_approximation_level_nm<3, 15, 4>();
}

//------------------------------------------------------------------------------
// Approximated Maps access

TEST_F(Pow2CachedPlainMaxApproximatorTest, dimensionsSwitchingWithTheLevel) {
  unsigned prev_w = 32, prev_h = 32;
  auto map = UnboundedPlainGridMap{cell_proto, {16, 16, 1}};
  auto approx = create_approximator(map);

  for (unsigned lvl = 0; lvl <= approx->max_approximation_level(); ++lvl) {
    auto &map = approx->map(lvl);
    ASSERT_LT(map.width(), prev_w);
    ASSERT_LT(map.height(), prev_h);
    prev_w = map.width();
    prev_h = map.height();
  }
  ASSERT_EQ(prev_w, 1);
  ASSERT_EQ(prev_h, 1);
}

TEST_F(Pow2CachedPlainMaxApproximatorTest, smokeTheCoarsestMapCheck) {
  auto map = UnboundedPlainGridMap{cell_proto, {16, 16, 1}};
  auto apxr = create_approximator(map);
  auto &coarse_map = apxr->map(apxr->max_approximation_level());

  ASSERT_EQ(DiscretePoint2D(0, 0), coarse_map.world_to_cell(100000, 100000));
  ASSERT_EQ(DiscretePoint2D(0, 0), coarse_map.world_to_cell(-100000, 100000));
  ASSERT_EQ(DiscretePoint2D(0, 0), coarse_map.world_to_cell(-100000, -100000));
  ASSERT_EQ(DiscretePoint2D(0, 0), coarse_map.world_to_cell(100000, -100000));
}


//------------------------------------------------------------------------------
// Map reading writing

TEST_F(Pow2CachedPlainMaxApproximatorTest, initDefaultValues) {
  auto map = UnboundedPlainGridMap{cell_proto, {3, 15, 1}};
  auto apxr = create_approximator(map);
  for (unsigned lvl = 0; lvl < apxr->max_approximation_level(); ++lvl) {
    auto &map = apxr->map(lvl);
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

TEST_F(Pow2CachedPlainMaxApproximatorTest, readWrite_16x16) {
  test_read_write_inside_map<UnboundedPlainGridMap, 16, 16>();
}

TEST_F(Pow2CachedPlainMaxApproximatorTest, readWrite_16x1) {
  test_read_write_inside_map<UnboundedPlainGridMap, 16, 1>();
}

TEST_F(Pow2CachedPlainMaxApproximatorTest, readWrite_1x16) {
  test_read_write_inside_map<UnboundedPlainGridMap, 1, 16>();
}

TEST_F(Pow2CachedPlainMaxApproximatorTest, readWrite_15x15) {
  test_read_write_inside_map<UnboundedPlainGridMap, 15, 15>();
}

TEST_F(Pow2CachedPlainMaxApproximatorTest, readWrite_33x33) {
  test_read_write_inside_map<UnboundedPlainGridMap, 33, 33>();
}

TEST_F(Pow2CachedPlainMaxApproximatorTest, readWrite_15x3) {
  test_read_write_inside_map<UnboundedPlainGridMap, 15, 3>();
}

TEST_F(Pow2CachedPlainMaxApproximatorTest, readWrite_3x15) {
  test_read_write_inside_map<UnboundedPlainGridMap, 3, 15>();
}

TEST_F(Pow2CachedPlainMaxApproximatorTest, readWriteUPGM_1000x1000) {
  test_read_write_inside_map<UnboundedPlainGridMap, 1000, 1000>();
}

TEST_F(Pow2CachedPlainMaxApproximatorTest, readWriteULTGM_1000x1000) {
  test_read_write_inside_map<UnboundedLazyTiledGridMap, 1000, 1000>();
}

TEST_F(Pow2CachedPlainMaxApproximatorTest, approximationLevelExtension) {
  auto map = UnboundedPlainGridMap{cell_proto, {16, 16, 1}};
  auto apxr = create_approximator(map);
  ASSERT_EQ(apxr->max_approximation_level(), 4);

  // init map
  auto val = AreaOccupancyObservation{true, Occupancy{128, 0},
                                      Point2D{0, 0}, 1};
  {
    auto updated_point = Point2D{23, 23};
    map.update(map.world_to_cell(updated_point), val);
    ASSERT_EQ(apxr->max_approximation_level(), 5);
  }

  {
    auto updated_point = Point2D{24, 24};
    map.update(map.world_to_cell(updated_point), val);
    ASSERT_EQ(apxr->max_approximation_level(), 6);
  }
}


// TODO: Reprojection policy tests
int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
