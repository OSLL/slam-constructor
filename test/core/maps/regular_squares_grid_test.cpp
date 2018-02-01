#include <gtest/gtest.h>

#include <memory>
#include <vector>
#include <algorithm>
#include <limits>

#include "../../../src/core/maps/regular_squares_grid.h"

//============================================================================//
//===                                 Tests                                ===//
//============================================================================//

class RSGSegmentRasterizationTest : public ::testing::Test {
protected:
  static constexpr double Grid_Scale = 0.1;
public:
  RSGSegmentRasterizationTest() : grid{100, 100, Grid_Scale} {}

protected: // methods

  Point2D cell_middle(const DiscretePoint2D& pnt) {
    return grid.cell_to_world(pnt);
  }

protected: // fields
  RegularSquaresGrid grid;
};

using DSegment = std::vector<DiscretePoint2D>;

//--------------------------//
// === Segment -> Cells === //

TEST_F(RSGSegmentRasterizationTest, samePoint) {
  ASSERT_EQ(DSegment({{7, 5}}),
            grid.world_to_cells({{0.75, 0.56}, {0.75, 0.56}}));
}

TEST_F(RSGSegmentRasterizationTest, sameCell) {
  ASSERT_EQ(DSegment({{7, 5}}),
            grid.world_to_cells({{0.71, 0.53}, {0.75, 0.51}}));
}

// 7x7 square, lines to each cell on edge, cw

TEST_F(RSGSegmentRasterizationTest, square00) {
  ASSERT_EQ(DSegment({{0, 0}, {0, 1}, {0, 2}, {0, 3}}),
            grid.world_to_cells({cell_middle({0, 0}), cell_middle({0, 3})}));
}

TEST_F(RSGSegmentRasterizationTest, square02) {
  ASSERT_EQ(DSegment({{0, 0}, {0, 1}, {1, 2}, {1, 3}}),
            grid.world_to_cells({cell_middle({0, 0}), cell_middle({1, 3})}));
}

TEST_F(RSGSegmentRasterizationTest, square04) {
  ASSERT_EQ(DSegment({{0, 0}, {0, 1}, {1, 1}, {1, 2}, {2, 2}, {2, 3}}),
            grid.world_to_cells({cell_middle({0, 0}), cell_middle({2, 3})}));
}

TEST_F(RSGSegmentRasterizationTest, square07) {
  ASSERT_EQ(DSegment({{0, 0}, {1, 1}, {2, 2}, {3, 3}}),
            grid.world_to_cells({cell_middle({0, 0}), cell_middle({3, 3})}));
}

TEST_F(RSGSegmentRasterizationTest, square10) {
  ASSERT_EQ(DSegment({{0, 0}, {1, 0}, {1, 1}, {2, 1}, {2, 2}, {3, 2}}),
            grid.world_to_cells({cell_middle({0, 0}), cell_middle({3, 2})}));
}

TEST_F(RSGSegmentRasterizationTest, square12) {
  ASSERT_EQ(DSegment({{0, 0}, {1, 0}, {2, 1}, {3, 1}}),
            grid.world_to_cells({cell_middle({0, 0}), cell_middle({3, 1})}));
}

TEST_F(RSGSegmentRasterizationTest, square15) {
  ASSERT_EQ(DSegment({{0, 0}, {1, 0}, {2, 0}, {3, 0}}),
            grid.world_to_cells({cell_middle({0, 0}), cell_middle({3, 0})}));
}

TEST_F(RSGSegmentRasterizationTest, square17) {
  ASSERT_EQ(DSegment({{0, 0}, {1, 0}, {2, -1}, {3, -1}}),
            grid.world_to_cells({cell_middle({0, 0}), cell_middle({3, -1})}));
}

TEST_F(RSGSegmentRasterizationTest, square19) {
  ASSERT_EQ(DSegment({{0, 0}, {1, 0}, {1, -1}, {2, -1}, {2, -2}, {3, -2}}),
            grid.world_to_cells({cell_middle({0, 0}), cell_middle({3, -2})}));
}

TEST_F(RSGSegmentRasterizationTest, square22) {
  ASSERT_EQ(DSegment({{0, 0}, {1, -1}, {2, -2}, {3, -3}}),
            grid.world_to_cells({cell_middle({0, 0}), cell_middle({3, -3})}));
}

TEST_F(RSGSegmentRasterizationTest, square25) {
  ASSERT_EQ(DSegment({{0, 0}, {0, -1}, {1, -1}, {1, -2}, {2, -2}, {2, -3}}),
            grid.world_to_cells({cell_middle({0, 0}), cell_middle({2, -3})}));
}

TEST_F(RSGSegmentRasterizationTest, square28) {
  ASSERT_EQ(DSegment({{0, 0}, {0, -1}, {1, -2}, {1, -3}}),
            grid.world_to_cells({cell_middle({0, 0}), cell_middle({1, -3})}));
}

TEST_F(RSGSegmentRasterizationTest, square30) {
  ASSERT_EQ(DSegment({{0, 0}, {0, -1}, {0, -2}, {0, -3}}),
            grid.world_to_cells({cell_middle({0, 0}), cell_middle({0, -3})}));
}

TEST_F(RSGSegmentRasterizationTest, square32) {
  ASSERT_EQ(DSegment({{0, 0}, {0, -1}, {-1, -2}, {-1, -3}}),
            grid.world_to_cells({cell_middle({0, 0}), cell_middle({-1, -3})}));
}

TEST_F(RSGSegmentRasterizationTest, square34) {
  ASSERT_EQ(DSegment({{0, 0}, {0, -1}, {-1, -1}, {-1, -2}, {-2, -2}, {-2, -3}}),
            grid.world_to_cells({cell_middle({0, 0}), cell_middle({-2, -3})}));
}

TEST_F(RSGSegmentRasterizationTest, square37) {
  ASSERT_EQ(DSegment({{0, 0}, {-1, -1}, {-2, -2}, {-3, -3}}),
            grid.world_to_cells({cell_middle({0, 0}), cell_middle({-3, -3})}));
}

TEST_F(RSGSegmentRasterizationTest, square40) {
  ASSERT_EQ(DSegment({{0, 0}, {-1, 0}, {-1, -1}, {-2, -1}, {-2, -2}, {-3, -2}}),
            grid.world_to_cells({cell_middle({0, 0}), cell_middle({-3, -2})}));
}

TEST_F(RSGSegmentRasterizationTest, square42) {
  ASSERT_EQ(DSegment({{0, 0}, {-1, 0}, {-2, -1}, {-3, -1}}),
            grid.world_to_cells({cell_middle({0, 0}), cell_middle({-3, -1})}));
}

TEST_F(RSGSegmentRasterizationTest, square45) {
  ASSERT_EQ(DSegment({{0, 0}, {-1, 0}, {-2, 0}, {-3, 0}}),
            grid.world_to_cells({cell_middle({0, 0}), cell_middle({-3, 0})}));
}

TEST_F(RSGSegmentRasterizationTest, square48) {
  ASSERT_EQ(DSegment({{0, 0}, {-1, 0}, {-2, 1}, {-3, 1}}),
            grid.world_to_cells({cell_middle({0, 0}), cell_middle({-3, 1})}));
}

TEST_F(RSGSegmentRasterizationTest, square50) {
  ASSERT_EQ(DSegment({{0, 0}, {-1, 0}, {-1, 1}, {-2, 1}, {-2, 2}, {-3, 2}}),
            grid.world_to_cells({cell_middle({0, 0}), cell_middle({-3, 2})}));
}

TEST_F(RSGSegmentRasterizationTest, square53) {
  ASSERT_EQ(DSegment({{0, 0}, {-1, 1}, {-2, 2}, {-3, 3}}),
            grid.world_to_cells({cell_middle({0, 0}), cell_middle({-3, 3})}));
}

TEST_F(RSGSegmentRasterizationTest, square56) {
  ASSERT_EQ(DSegment({{0, 0}, {0, 1}, {-1, 1}, {-1, 2}, {-2, 2}, {-2, 3}}),
            grid.world_to_cells({cell_middle({0, 0}), cell_middle({-2, 3})}));
}

TEST_F(RSGSegmentRasterizationTest, square58) {
  ASSERT_EQ(DSegment({{0, 0}, {0, 1}, {-1, 2}, {-1, 3}}),
            grid.world_to_cells({cell_middle({0, 0}), cell_middle({-1, 3})}));
}

// horiz
TEST_F(RSGSegmentRasterizationTest, horizFwdStraight) {
  ASSERT_EQ(DSegment({{7, 5}, {8, 5}, {9, 5}, {10, 5}}),
            grid.world_to_cells({{0.71, 0.53}, {1.02, 0.53}}));
}

TEST_F(RSGSegmentRasterizationTest, horizFwdUp) {
  ASSERT_EQ(DSegment({{-4, -6}, {-3, -6}, {-2, -6}, {-1, -6}}),
            grid.world_to_cells({{-0.37, -0.56}, {-0.05, -0.51}}));
}

TEST_F(RSGSegmentRasterizationTest, horizFwdDown) {
  ASSERT_EQ(DSegment({{7, 5}, {8, 5}, {9, 5}, {10, 5}}),
            grid.world_to_cells({{0.71, 0.57}, {1.02, 0.52}}));
}

TEST_F(RSGSegmentRasterizationTest, horizBwdStraight) {
  ASSERT_EQ(DSegment({{10, 5}, {9, 5}, {8, 5}, {7, 5}}),
            grid.world_to_cells({{1.02, 0.53}, {0.71, 0.53}}));
}

//vert
TEST_F(RSGSegmentRasterizationTest, vertFwdStraight) {
  ASSERT_EQ(DSegment({{7, 5}, {7, 6}, {7, 7}, {7, 8}}),
            grid.world_to_cells({{0.73, 0.52}, {0.73, 0.85}}));
}

TEST_F(RSGSegmentRasterizationTest, vertFwdLeft) {
  ASSERT_EQ(DSegment({{7, 5}, {7, 6}, {7, 7}, {7, 8}}),
            grid.world_to_cells({{0.77, 0.51}, {0.73, 0.87}}));
}

TEST_F(RSGSegmentRasterizationTest, vertFwdRight) {
  ASSERT_EQ(DSegment({{7, 5}, {7, 6}, {7, 7}, {7, 8}}),
            grid.world_to_cells({{0.71, 0.51}, {0.75, 0.83}}));
}

TEST_F(RSGSegmentRasterizationTest, vertBwdStraight) {
  ASSERT_EQ(DSegment({{7, 8}, {7, 7}, {7, 6}, {7, 5}}),
            grid.world_to_cells({{0.73, 0.89}, {0.73, 0.52}}));
}

TEST_F(RSGSegmentRasterizationTest, vertSlimFwd) {
  ASSERT_EQ(DSegment({{0, 0}, {0, 1}, {0, 2}, {0, 3},
                      {1, 3}, {1, 4}, {1, 5}, {1, 6}}),
            grid.world_to_cells({cell_middle({0, 0}), cell_middle({1, 6})}));
}

//diag
TEST_F(RSGSegmentRasterizationTest, diag45FwdPure) {
  ASSERT_EQ(grid.world_to_cells({{0.51, 0.71}, {0.81, 1.01}}),
            DSegment({{5, 7}, {6, 8}, {7, 9}, {8, 10}}));
}

TEST_F(RSGSegmentRasterizationTest, diag45BwdPure) {
  ASSERT_EQ(grid.world_to_cells({{0.81, 1.01}, {0.51, 0.71}}),
            DSegment({{8, 10}, {7, 9}, {6, 8}, {5, 7}}));
}

TEST_F(RSGSegmentRasterizationTest, diagFwdShiftDown) {
  ASSERT_EQ(grid.world_to_cells({{-0.12, -0.17}, {0.17, 0.12}}),
            DSegment({{-2, -2}, {-1, -2}, {-1, -1},
                      {0, -1}, {0, 0}, {1, 0}, {1, 1}}));
}

TEST_F(RSGSegmentRasterizationTest, diagBwdShiftDown) {
  ASSERT_EQ(grid.world_to_cells({{0.17, 0.12}, {-0.12, -0.17}}),
            DSegment({{1, 1}, {1, 0}, {0, 0},
                      {0, -1}, {-1, -1}, {-1, -2}, {-2, -2}}));
}

TEST_F(RSGSegmentRasterizationTest, diagFwdShiftUp) {
  ASSERT_EQ(grid.world_to_cells({{-0.17, -0.12}, {0.12, 0.17}}),
            DSegment({{-2, -2}, {-2, -1}, {-1, -1},
                      {-1, 0}, {0, 0}, {0, 1}, {1, 1}}));
}

TEST_F(RSGSegmentRasterizationTest, diagBwdShiftUp) {
  ASSERT_EQ(grid.world_to_cells({{0.12, 0.17}, {-0.17, -0.12}}),
            DSegment({{1, 1}, {0, 1}, {0, 0},
                      {-1, 0}, {-1, -1}, {-2, -1}, {-2, -2}}));
}

TEST_F(RSGSegmentRasterizationTest, diagFwdThroughCenter) {
  ASSERT_EQ(grid.world_to_cells({{-0.17, -0.12}, {0.17, 0.12}}),
            DSegment({{-2, -2}, {-2, -1}, {-1, -1},
                      {0, 0}, {1, 0}, {1, 1}}));
}

TEST_F(RSGSegmentRasterizationTest, diagBwdThroughCenter) {
  ASSERT_EQ(grid.world_to_cells({{0.17, 0.12}, {-0.17, -0.12},}),
            DSegment({{1, 1}, {1, 0}, {0, 0},
                      {-1, -1}, {-2, -1}, {-2, -2}}));
}

// coincide with coordinate line

TEST_F(RSGSegmentRasterizationTest, horizontalLineAlinedFwd) {
  ASSERT_EQ(grid.world_to_cells({{0, 0}, {0, 0.45}}),
            DSegment({{0, 0}, {0, 1}, {0, 2}, {0, 3}, {0, 4}}));
}

TEST_F(RSGSegmentRasterizationTest, horizontalLineAlinedBwd) {
  ASSERT_EQ(grid.world_to_cells({{0, 0}, {0, -0.35}}),
            DSegment({{0, 0}, {0, -1}, {0, -2}, {0, -3}, {0, -4}}));
}

TEST_F(RSGSegmentRasterizationTest, verticalLineAlinedFwd) {
  ASSERT_EQ(grid.world_to_cells({{0, 0}, {0.45, 0}}),
            DSegment({{0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}}));
}

TEST_F(RSGSegmentRasterizationTest, verticalLineAlinedBwd) {
  ASSERT_EQ(grid.world_to_cells({{0, 0}, {-0.35, 0}}),
            DSegment({{0, 0}, {-1, 0}, {-2, 0}, {-3, 0}, {-4, 0}}));
}

//============================================================================//

class RSGInfinityScalingTest : public ::testing::Test {
protected:
  static_assert(std::numeric_limits<double>::has_infinity,
                "Infinity is not defined for doubles.");
  static constexpr double Inf = std::numeric_limits<double>::infinity();
protected:
  RSGInfinityScalingTest() : grid{1, 1, Inf} {}

  void test_grid_access(double x, double y) {
    ASSERT_EQ(DiscretePoint2D(0, 0), grid.world_to_cell(x, y));
  }
protected: // fields
  RegularSquaresGrid grid;
};

TEST_F(RSGInfinityScalingTest, access1stQ) {
  test_grid_access(0, 0);
  test_grid_access(std::numeric_limits<double>::max(),
                  std::numeric_limits<double>::max());
  /* FIXME
  test_grid_access(std::numeric_limits<double>::infinity(),
                  std::numeric_limits<double>::infinity());
  */
}

TEST_F(RSGInfinityScalingTest, access2ndQ) {
  test_grid_access(-5, 3);
  test_grid_access(std::numeric_limits<double>::lowest(),
                   std::numeric_limits<double>::max());
  /* FIXME
  test_grid_access(-std::numeric_limits<double>::infinity(),
                   std::numeric_limits<double>::infinity());
  */
}

TEST_F(RSGInfinityScalingTest, access3rdQ) {
  test_grid_access(-5, -3);
  test_grid_access(std::numeric_limits<double>::lowest(),
                  std::numeric_limits<double>::lowest());
  /* FIXME
  test_grid_access(-std::numeric_limits<double>::infinity(),
                   -std::numeric_limits<double>::infinity());
  */
}

TEST_F(RSGInfinityScalingTest, access4thQ) {
  test_grid_access(5, -3);
  test_grid_access(std::numeric_limits<double>::max(),
                  std::numeric_limits<double>::lowest());
  /* FIXME
  test_grid_access(std::numeric_limits<double>::infinity(),
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
