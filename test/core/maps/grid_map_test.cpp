#include <gtest/gtest.h>

#include <memory>
#include <vector>
#include <algorithm>
#include <limits>

#include "../mock_grid_cell.h"

#include "../../../src/core/maps/grid_map.h"

//============================================================================//
//===                                 Mocks                                ===//
//============================================================================//

class MockGridMap : public GridMap {
public:
  MockGridMap(const GridMapParams &gmp) :
    GridMap{std::make_shared<MockGridCell>(), gmp}, _cells(gmp.height_cells) {
    for (auto &row : _cells) {
      row.reserve(GridMap::width());
      std::generate_n(std::back_inserter(row), GridMap::width(),
                      [this](){ return new_cell(); });
    }
  }

  const GridCell &operator[](const Coord& c) const override {
    assert(has_cell(c));
    auto coord = external2internal(c);
    return *_cells[coord.y][coord.x];
  }

private: // fields
  std::vector<std::vector<std::unique_ptr<GridCell>>> _cells;
};

//============================================================================//
//===                                 Tests                                ===//
//============================================================================//

class GridMapRasterizationTest : public ::testing::Test {
protected:
  static constexpr double Grid_Scale = 0.1;
public:
  GridMapRasterizationTest() : map{{100, 100, Grid_Scale}} {}

protected: // methods

  Point2D cell_middle(const DiscretePoint2D& pnt) {
    return map.cell_to_world(pnt);
  }

protected: // fields
  MockGridMap map;
};

using DSegment = std::vector<DiscretePoint2D>;

//--------------------------//
// === Segment -> Cells === //

TEST_F(GridMapRasterizationTest, samePoint) {
  ASSERT_EQ(DSegment({{7, 5}}),
            map.world_to_cells({{0.75, 0.56}, {0.75, 0.56}}));
}

TEST_F(GridMapRasterizationTest, sameCell) {
  ASSERT_EQ(DSegment({{7, 5}}),
            map.world_to_cells({{0.71, 0.53}, {0.75, 0.51}}));
}

// 7x7 square, lines to each cell on edge, cw

TEST_F(GridMapRasterizationTest, square00) {
  ASSERT_EQ(DSegment({{0, 0}, {0, 1}, {0, 2}, {0, 3}}),
            map.world_to_cells({cell_middle({0, 0}), cell_middle({0, 3})}));
}

TEST_F(GridMapRasterizationTest, square02) {
  ASSERT_EQ(DSegment({{0, 0}, {0, 1}, {1, 2}, {1, 3}}),
            map.world_to_cells({cell_middle({0, 0}), cell_middle({1, 3})}));
}

TEST_F(GridMapRasterizationTest, square04) {
  ASSERT_EQ(DSegment({{0, 0}, {0, 1}, {1, 1}, {1, 2}, {2, 2}, {2, 3}}),
            map.world_to_cells({cell_middle({0, 0}), cell_middle({2, 3})}));
}

TEST_F(GridMapRasterizationTest, square07) {
  ASSERT_EQ(DSegment({{0, 0}, {1, 1}, {2, 2}, {3, 3}}),
            map.world_to_cells({cell_middle({0, 0}), cell_middle({3, 3})}));
}

TEST_F(GridMapRasterizationTest, square10) {
  ASSERT_EQ(DSegment({{0, 0}, {1, 0}, {1, 1}, {2, 1}, {2, 2}, {3, 2}}),
            map.world_to_cells({cell_middle({0, 0}), cell_middle({3, 2})}));
}

TEST_F(GridMapRasterizationTest, square12) {
  ASSERT_EQ(DSegment({{0, 0}, {1, 0}, {2, 1}, {3, 1}}),
            map.world_to_cells({cell_middle({0, 0}), cell_middle({3, 1})}));
}

TEST_F(GridMapRasterizationTest, square15) {
  ASSERT_EQ(DSegment({{0, 0}, {1, 0}, {2, 0}, {3, 0}}),
            map.world_to_cells({cell_middle({0, 0}), cell_middle({3, 0})}));
}

TEST_F(GridMapRasterizationTest, square17) {
  ASSERT_EQ(DSegment({{0, 0}, {1, 0}, {2, -1}, {3, -1}}),
            map.world_to_cells({cell_middle({0, 0}), cell_middle({3, -1})}));
}

TEST_F(GridMapRasterizationTest, square19) {
  ASSERT_EQ(DSegment({{0, 0}, {1, 0}, {1, -1}, {2, -1}, {2, -2}, {3, -2}}),
            map.world_to_cells({cell_middle({0, 0}), cell_middle({3, -2})}));
}

TEST_F(GridMapRasterizationTest, square22) {
  ASSERT_EQ(DSegment({{0, 0}, {1, -1}, {2, -2}, {3, -3}}),
            map.world_to_cells({cell_middle({0, 0}), cell_middle({3, -3})}));
}

TEST_F(GridMapRasterizationTest, square25) {
  ASSERT_EQ(DSegment({{0, 0}, {0, -1}, {1, -1}, {1, -2}, {2, -2}, {2, -3}}),
            map.world_to_cells({cell_middle({0, 0}), cell_middle({2, -3})}));
}

TEST_F(GridMapRasterizationTest, square28) {
  ASSERT_EQ(DSegment({{0, 0}, {0, -1}, {1, -2}, {1, -3}}),
            map.world_to_cells({cell_middle({0, 0}), cell_middle({1, -3})}));
}

TEST_F(GridMapRasterizationTest, square30) {
  ASSERT_EQ(DSegment({{0, 0}, {0, -1}, {0, -2}, {0, -3}}),
            map.world_to_cells({cell_middle({0, 0}), cell_middle({0, -3})}));
}

TEST_F(GridMapRasterizationTest, square32) {
  ASSERT_EQ(DSegment({{0, 0}, {0, -1}, {-1, -2}, {-1, -3}}),
            map.world_to_cells({cell_middle({0, 0}), cell_middle({-1, -3})}));
}

TEST_F(GridMapRasterizationTest, square34) {
  ASSERT_EQ(DSegment({{0, 0}, {0, -1}, {-1, -1}, {-1, -2}, {-2, -2}, {-2, -3}}),
            map.world_to_cells({cell_middle({0, 0}), cell_middle({-2, -3})}));
}

TEST_F(GridMapRasterizationTest, square37) {
  ASSERT_EQ(DSegment({{0, 0}, {-1, -1}, {-2, -2}, {-3, -3}}),
            map.world_to_cells({cell_middle({0, 0}), cell_middle({-3, -3})}));
}

TEST_F(GridMapRasterizationTest, square40) {
  ASSERT_EQ(DSegment({{0, 0}, {-1, 0}, {-1, -1}, {-2, -1}, {-2, -2}, {-3, -2}}),
            map.world_to_cells({cell_middle({0, 0}), cell_middle({-3, -2})}));
}

TEST_F(GridMapRasterizationTest, square42) {
  ASSERT_EQ(DSegment({{0, 0}, {-1, 0}, {-2, -1}, {-3, -1}}),
            map.world_to_cells({cell_middle({0, 0}), cell_middle({-3, -1})}));
}

TEST_F(GridMapRasterizationTest, square45) {
  ASSERT_EQ(DSegment({{0, 0}, {-1, 0}, {-2, 0}, {-3, 0}}),
            map.world_to_cells({cell_middle({0, 0}), cell_middle({-3, 0})}));
}

TEST_F(GridMapRasterizationTest, square48) {
  ASSERT_EQ(DSegment({{0, 0}, {-1, 0}, {-2, 1}, {-3, 1}}),
            map.world_to_cells({cell_middle({0, 0}), cell_middle({-3, 1})}));
}

TEST_F(GridMapRasterizationTest, square50) {
  ASSERT_EQ(DSegment({{0, 0}, {-1, 0}, {-1, 1}, {-2, 1}, {-2, 2}, {-3, 2}}),
            map.world_to_cells({cell_middle({0, 0}), cell_middle({-3, 2})}));
}

TEST_F(GridMapRasterizationTest, square53) {
  ASSERT_EQ(DSegment({{0, 0}, {-1, 1}, {-2, 2}, {-3, 3}}),
            map.world_to_cells({cell_middle({0, 0}), cell_middle({-3, 3})}));
}

TEST_F(GridMapRasterizationTest, square56) {
  ASSERT_EQ(DSegment({{0, 0}, {0, 1}, {-1, 1}, {-1, 2}, {-2, 2}, {-2, 3}}),
            map.world_to_cells({cell_middle({0, 0}), cell_middle({-2, 3})}));
}

TEST_F(GridMapRasterizationTest, square58) {
  ASSERT_EQ(DSegment({{0, 0}, {0, 1}, {-1, 2}, {-1, 3}}),
            map.world_to_cells({cell_middle({0, 0}), cell_middle({-1, 3})}));
}

// horiz
TEST_F(GridMapRasterizationTest, horizFwdStraight) {
  ASSERT_EQ(DSegment({{7, 5}, {8, 5}, {9, 5}, {10, 5}}),
            map.world_to_cells({{0.71, 0.53}, {1.02, 0.53}}));
}

TEST_F(GridMapRasterizationTest, horizFwdUp) {
  ASSERT_EQ(DSegment({{-4, -6}, {-3, -6}, {-2, -6}, {-1, -6}}),
            map.world_to_cells({{-0.37, -0.56}, {-0.05, -0.51}}));
}

TEST_F(GridMapRasterizationTest, horizFwdDown) {
  ASSERT_EQ(DSegment({{7, 5}, {8, 5}, {9, 5}, {10, 5}}),
            map.world_to_cells({{0.71, 0.57}, {1.02, 0.52}}));
}

TEST_F(GridMapRasterizationTest, horizBwdStraight) {
  ASSERT_EQ(DSegment({{10, 5}, {9, 5}, {8, 5}, {7, 5}}),
            map.world_to_cells({{1.02, 0.53}, {0.71, 0.53}}));
}

//vert
TEST_F(GridMapRasterizationTest, vertFwdStraight) {
  ASSERT_EQ(DSegment({{7, 5}, {7, 6}, {7, 7}, {7, 8}}),
            map.world_to_cells({{0.73, 0.52}, {0.73, 0.85}}));
}

TEST_F(GridMapRasterizationTest, vertFwdLeft) {
  ASSERT_EQ(DSegment({{7, 5}, {7, 6}, {7, 7}, {7, 8}}),
            map.world_to_cells({{0.77, 0.51}, {0.73, 0.87}}));
}

TEST_F(GridMapRasterizationTest, vertFwdRight) {
  ASSERT_EQ(DSegment({{7, 5}, {7, 6}, {7, 7}, {7, 8}}),
            map.world_to_cells({{0.71, 0.51}, {0.75, 0.83}}));
}

TEST_F(GridMapRasterizationTest, vertBwdStraight) {
  ASSERT_EQ(DSegment({{7, 8}, {7, 7}, {7, 6}, {7, 5}}),
            map.world_to_cells({{0.73, 0.89}, {0.73, 0.52}}));
}

TEST_F(GridMapRasterizationTest, vertSlimFwd) {
  ASSERT_EQ(DSegment({{0, 0}, {0, 1}, {0, 2}, {0, 3},
                      {1, 3}, {1, 4}, {1, 5}, {1, 6}}),
            map.world_to_cells({cell_middle({0, 0}), cell_middle({1, 6})}));
}

//diag
TEST_F(GridMapRasterizationTest, diag45FwdPure) {
  ASSERT_EQ(map.world_to_cells({{0.51, 0.71}, {0.81, 1.01}}),
            DSegment({{5, 7}, {6, 8}, {7, 9}, {8, 10}}));
}

TEST_F(GridMapRasterizationTest, diag45BwdPure) {
  ASSERT_EQ(map.world_to_cells({{0.81, 1.01}, {0.51, 0.71}}),
            DSegment({{8, 10}, {7, 9}, {6, 8}, {5, 7}}));
}

TEST_F(GridMapRasterizationTest, diagFwdShiftDown) {
  ASSERT_EQ(map.world_to_cells({{-0.12, -0.17}, {0.17, 0.12}}),
            DSegment({{-2, -2}, {-1, -2}, {-1, -1},
                      {0, -1}, {0, 0}, {1, 0}, {1, 1}}));
}

TEST_F(GridMapRasterizationTest, diagBwdShiftDown) {
  ASSERT_EQ(map.world_to_cells({{0.17, 0.12}, {-0.12, -0.17}}),
            DSegment({{1, 1}, {1, 0}, {0, 0},
                      {0, -1}, {-1, -1}, {-1, -2}, {-2, -2}}));
}

TEST_F(GridMapRasterizationTest, diagFwdShiftUp) {
  ASSERT_EQ(map.world_to_cells({{-0.17, -0.12}, {0.12, 0.17}}),
            DSegment({{-2, -2}, {-2, -1}, {-1, -1},
                      {-1, 0}, {0, 0}, {0, 1}, {1, 1}}));
}

TEST_F(GridMapRasterizationTest, diagBwdShiftUp) {
  ASSERT_EQ(map.world_to_cells({{0.12, 0.17}, {-0.17, -0.12}}),
            DSegment({{1, 1}, {0, 1}, {0, 0},
                      {-1, 0}, {-1, -1}, {-2, -1}, {-2, -2}}));
}

TEST_F(GridMapRasterizationTest, diagFwdThroughCenter) {
  ASSERT_EQ(map.world_to_cells({{-0.17, -0.12}, {0.17, 0.12}}),
            DSegment({{-2, -2}, {-2, -1}, {-1, -1},
                      {0, 0}, {1, 0}, {1, 1}}));
}

TEST_F(GridMapRasterizationTest, diagBwdThroughCenter) {
  ASSERT_EQ(map.world_to_cells({{0.17, 0.12}, {-0.17, -0.12},}),
            DSegment({{1, 1}, {1, 0}, {0, 0},
                      {-1, -1}, {-2, -1}, {-2, -2}}));
}

// coincide with coordinate line

TEST_F(GridMapRasterizationTest, horizontalLineAlinedFwd) {
  ASSERT_EQ(map.world_to_cells({{0, 0}, {0, 0.45}}),
            DSegment({{0, 0}, {0, 1}, {0, 2}, {0, 3}, {0, 4}}));
}

TEST_F(GridMapRasterizationTest, horizontalLineAlinedBwd) {
  ASSERT_EQ(map.world_to_cells({{0, 0}, {0, -0.35}}),
            DSegment({{0, 0}, {0, -1}, {0, -2}, {0, -3}, {0, -4}}));
}

TEST_F(GridMapRasterizationTest, verticalLineAlinedFwd) {
  ASSERT_EQ(map.world_to_cells({{0, 0}, {0.45, 0}}),
            DSegment({{0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}}));
}

TEST_F(GridMapRasterizationTest, verticalLineAlinedBwd) {
  ASSERT_EQ(map.world_to_cells({{0, 0}, {-0.35, 0}}),
            DSegment({{0, 0}, {-1, 0}, {-2, 0}, {-3, 0}, {-4, 0}}));
}

//============================================================================//

class GridMapInfinityScalingTest : public ::testing::Test {
protected:
  static_assert(std::numeric_limits<double>::has_infinity);
  static constexpr double Inf = std::numeric_limits<double>::infinity();
protected:
  GridMapInfinityScalingTest() : map{{1, 1, Inf}} {}

  void test_map_access(double x, double y) {
    ASSERT_EQ(DiscretePoint2D(0, 0), map.world_to_cell(x, y));
  }
protected: // fields
  MockGridMap map;
};

TEST_F(GridMapInfinityScalingTest, access1stQ) {
  test_map_access(0, 0);
  test_map_access(std::numeric_limits<double>::max(),
                  std::numeric_limits<double>::max());
  /* FIXME
  test_map_access(std::numeric_limits<double>::infinity(),
                  std::numeric_limits<double>::infinity());
  */
}

TEST_F(GridMapInfinityScalingTest, access2ndQ) {
  test_map_access(-5, 3);
  test_map_access(std::numeric_limits<double>::lowest(),
                  std::numeric_limits<double>::max());
  /* FIXME
  test_map_access(-std::numeric_limits<double>::infinity(),
                  std::numeric_limits<double>::infinity());
  */
}

TEST_F(GridMapInfinityScalingTest, access3rdQ) {
  test_map_access(-5, -3);
  test_map_access(std::numeric_limits<double>::lowest(),
                  std::numeric_limits<double>::lowest());
  /* FIXME
  test_map_access(-std::numeric_limits<double>::infinity(),
                  -std::numeric_limits<double>::infinity());
  */
}

TEST_F(GridMapInfinityScalingTest, access4thQ) {
  test_map_access(5, -3);
  test_map_access(std::numeric_limits<double>::max(),
                  std::numeric_limits<double>::lowest());
  /* FIXME
  test_map_access(std::numeric_limits<double>::infinity(),
                  -std::numeric_limits<double>::infinity());
  */
}

//============================================================================//
//============================================================================//
//============================================================================//

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
