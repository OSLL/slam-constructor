#include <gtest/gtest.h>

#include <memory>
#include <vector>
#include <algorithm>
#include <limits>

#include "../../../src/core/maps/regular_squares_grid.h"

class Directions {
private:
  constexpr static int Left_Id  = 1 << 0;
  constexpr static int Right_Id = 1 << 1;
  constexpr static int Up_Id    = 1 << 2;
  constexpr static int Down_Id  = 1 << 3;
public:
  Directions& set_left()  { _data |= Left_Id; return *this; }
  Directions& set_right() { _data |= Right_Id; return *this; }
  Directions& set_up()    { _data |= Up_Id; return *this; }
  Directions& set_down()  { _data |= Down_Id; return *this; }

  bool left() const  { return _data & Left_Id; }
  bool right() const { return _data & Right_Id; }
  bool up() const    { return _data & Up_Id; }
  bool down() const  { return _data & Down_Id; }

private:
  int _data = 0;
};

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

class RSGRectangleRasterizationTest : public ::testing::Test {
protected:
  static constexpr double Grid_Scale = 0.5; // use 0.5 to prevent FP erros
  static constexpr double Cell_Len = Grid_Scale;
  static constexpr double Half_Cell_Len = Cell_Len / 2;
  static constexpr double Quat_Cell_Len = Half_Cell_Len / 2;
public:
  RSGRectangleRasterizationTest() : grid{100, 100, Grid_Scale} {}

protected: // types
  using AreaId = RegularSquaresGrid::Coord;
  using AreaIds = std::set<AreaId>;
protected: // methods

  AreaIds area_to_ids(const Point2D &center, double side_len) {
    auto half_side = side_len / 2;
    auto rect = Rectangle{-half_side, half_side, -half_side, half_side};

    auto raw_area_ids = grid.coords_in_area(rect.move_to_center(center));
    return AreaIds{raw_area_ids.begin(), raw_area_ids.end()};
  }

  AreaIds area_to_ids(const Point2D &offset,
                      double bot_len, double top_len,
                      double left_len, double right_len) {
    auto rect = Rectangle{-bot_len + offset.y, top_len + offset.y,
                          -left_len + offset.x, right_len + offset.x};
    auto raw_area_ids = grid.coords_in_area(rect);
    return AreaIds{raw_area_ids.begin(), raw_area_ids.end()};
  }

   /*
   *  ...................
   *  .     .     .     .
   *  .     .     .     .
   *  .  @@@@@@@@@@@@@  .
   *  .  @  .  ^  .  @  .
   *  .  @  .  |  .  @  .
   *  ...@.....|.....@...
   *  .  @  .  |  .  @  .
   *  .  @  . *** .  @  .
   *  .  @<---* *--->@  .
   *  .  @  . *** .  @  .
   *  .  @  .  |  .  @  .
   *  ...@.....|.....@...
   *  .  @  .  |  .  @  .
   *  .  @  .  V  .  @  .
   *  .  @@@@@@@@@@@@@  .
   *  .     .     .     .
   *  .     .     .     .
   *  ...................
   */
  void test_area_exceeding(const Directions &dirs) {
    auto area_id = AreaId{};
    for (area_id.x = -1; area_id.x <= 1; ++area_id.x) {
      for (area_id.y = -1; area_id.y <= 1; ++area_id.y) {
        // setup params
        auto area_offset = grid.cell_to_world(area_id);
        auto left = Quat_Cell_Len, right = Quat_Cell_Len,
             top = Quat_Cell_Len, bot = Quat_Cell_Len;

        int expected_d_x_start = 0, expected_d_x_end = 0,
            expected_d_y_start = 0, expected_d_y_end = 0;
        if (dirs.left()) {
          area_offset.x -= Quat_Cell_Len;
          left += Half_Cell_Len;
          expected_d_x_start = -1;
        }
        if (dirs.right()) {
          area_offset.x += Quat_Cell_Len;
          right += Half_Cell_Len;
          expected_d_x_end = 1;
        }
        if (dirs.down()) {
          area_offset.y -= Quat_Cell_Len;
          bot += Half_Cell_Len;
          expected_d_y_start = -1;
        }
        if (dirs.up()) {
          area_offset.y += Quat_Cell_Len;
          top += Half_Cell_Len;
          expected_d_y_end = 1;
        }

        // generate expected
        auto expected = AreaIds{};
        for (int d_x = expected_d_x_start; d_x <= expected_d_x_end; ++d_x) {
          for (int d_y = expected_d_y_start; d_y <= expected_d_y_end; ++d_y) {
            expected.emplace(area_id.x + d_x, area_id.y + d_y);
          }
        }

        // generate actual
        auto actual = area_to_ids(area_offset, bot, top, left, right);
        // check area
        ASSERT_EQ(expected, actual);
      }
    }
  }

  /*
   *   .     .     .     .
   *   .     .     .     .
   * ..@@@@@@@@@@@@@@@@@@@..
   *   @     .  ^  .     @
   *   @     .  |  .     @
   *   @  *************  @
   *   @  *  .     .  *  @
   *   @  *  .     .  *  @
   * ..@..*...........*..@..
   *   @  *  .     .  *  @
   *   @  *  .     .  *  @
   *   @<-*  .     .  *->@
   *   @  *  .     .  *  @
   *   @  *  .     .  *  @
   * ..@..*...........*..@..
   *   @  *  .     .  *  @
   *   @  *  .     .  *  @
   *   @  *************  @
   *   @     .  |  .     @
   *   @     .  V  .     @
   * ..@@@@@@@@@@@@@@@@@@@..
   *   .     .     .     .
   *   .     .     .     .
   */
  void test_area_alignement(const Directions &dirs) {
    auto area_id = AreaId{};
    for (area_id.x = -1; area_id.x <= 1; ++area_id.x) {
      for (area_id.y = -1; area_id.y <= 1; ++area_id.y) {
        int d_x_limit = 1, d_y_limit = 1;
        // generate expected
        auto left = Cell_Len, right = Cell_Len, top = Cell_Len, bot = Cell_Len;
        if (dirs.left())  { left  += Half_Cell_Len; }
        if (dirs.down())   { bot   += Half_Cell_Len; }

        if (dirs.right()) {
          ++d_x_limit;
          right += Half_Cell_Len;
        }

        if (dirs.up()) {
          ++d_y_limit;
          top += Half_Cell_Len;
        }

        auto expected = AreaIds{};
        for (int d_x = -1; d_x <= d_x_limit; ++d_x) {
          for (int d_y = -1; d_y <= d_y_limit; ++d_y) {
            expected.emplace(area_id.x + d_x, area_id.y + d_y);
          }
        }

        // generate actual
        auto actual = area_to_ids(grid.cell_to_world(area_id),
                                  bot, top, left, right);
        // check area
        ASSERT_EQ(expected, actual);
      }
    }
  }

protected: // fields
  RegularSquaresGrid grid;
};

bool operator<(const RegularSquaresGrid::Coord &c1,
               const RegularSquaresGrid::Coord &c2) {
  if (c1.x == c2.x) { return c1.y < c2.y; }
  return c1.x < c2.x;
}

TEST_F(RSGRectangleRasterizationTest, emptyAreaInsideCell) {
  auto area_id = AreaId{};
  for (area_id.x = -1; area_id.x <= 1; ++area_id.x) {
    for (area_id.y = -1; area_id.y <= 1; ++area_id.y) {
      ASSERT_EQ(area_to_ids({grid.cell_to_world(area_id)}, 0),
                AreaIds{area_id});
    }
  }
}

TEST_F(RSGRectangleRasterizationTest, emptyAreaOnBorders) {
  // left bot
  ASSERT_EQ(area_to_ids({0, 0}, 0), (AreaIds{{0, 0}}));

  // left
  ASSERT_EQ(area_to_ids({0, Half_Cell_Len}, 0), (AreaIds{{0, 0}}));

  // left top
  ASSERT_EQ(area_to_ids({0, Cell_Len}, 0), (AreaIds{{0, 1}}));

  // top
  ASSERT_EQ(area_to_ids({Half_Cell_Len, Cell_Len}, 0), (AreaIds{{0, 1}}));

  // right top
  ASSERT_EQ(area_to_ids({Cell_Len, Cell_Len}, 0), (AreaIds{{1, 1}}));

  // right
  ASSERT_EQ(area_to_ids({Cell_Len, Half_Cell_Len}, 0), (AreaIds{{1, 0}}));

  // right bot
  ASSERT_EQ(area_to_ids({Cell_Len, 0}, 0), (AreaIds{{1, 0}}));

  // bot
  ASSERT_EQ(area_to_ids({Half_Cell_Len, 0}, 0), (AreaIds{{0, 0}}));
}

//----------------------------------------------------------------------------//
//--- An area exceeds a Cell ---//

TEST_F(RSGRectangleRasterizationTest, areaDoesNotExceedCell) {
  test_area_exceeding(Directions{});
}

TEST_F(RSGRectangleRasterizationTest, areaExceedsCellLeft) {
  test_area_exceeding(Directions{}.set_left());
}

TEST_F(RSGRectangleRasterizationTest, areaExceedsCellRight) {
  test_area_exceeding(Directions{}.set_right());
}

TEST_F(RSGRectangleRasterizationTest, areaExceedsCellRightLeft) {
  test_area_exceeding(Directions{}.set_right().set_left());
}

TEST_F(RSGRectangleRasterizationTest, areaExceedsCellUp) {
  test_area_exceeding(Directions{}.set_up());
}

TEST_F(RSGRectangleRasterizationTest, areaExceedsCellUpLeft) {
  test_area_exceeding(Directions{}.set_up().set_left());
}

TEST_F(RSGRectangleRasterizationTest, areaExceedsCellUpRight) {
  test_area_exceeding(Directions{}.set_up().set_right());
}

TEST_F(RSGRectangleRasterizationTest, areaExceedsCellUpRightLeft) {
  test_area_exceeding(Directions{}.set_up().set_right().set_left());
}

TEST_F(RSGRectangleRasterizationTest, areaExceedsCellDown) {
  test_area_exceeding(Directions{}.set_down());
}

TEST_F(RSGRectangleRasterizationTest, areaExceedsCellDownLeft) {
  test_area_exceeding(Directions{}.set_down().set_left());
}

TEST_F(RSGRectangleRasterizationTest, areaExceedsCellDownRight) {
  test_area_exceeding(Directions{}.set_down().set_right());
}

TEST_F(RSGRectangleRasterizationTest, areaExceedsCellDownRightLeft) {
  test_area_exceeding(Directions{}.set_down().set_right().set_left());
}

TEST_F(RSGRectangleRasterizationTest, areaExceedsCellDownUp) {
  test_area_exceeding(Directions{}.set_down().set_up());
}

TEST_F(RSGRectangleRasterizationTest, areaExceedsCellDownUpLeft) {
  test_area_exceeding(Directions{}.set_down().set_up().set_left());
}

TEST_F(RSGRectangleRasterizationTest, areaExceedsCellDownUpRight) {
  test_area_exceeding(Directions{}.set_down().set_up().set_right());
}

TEST_F(RSGRectangleRasterizationTest, areaExceedsCellDownUpRightLeft) {
  test_area_exceeding(Directions{}.set_down().set_up().set_right().set_left());
}

//----------------------------------------------------------------------------//
//--- An area is aligned with cell's border ---//

TEST_F(RSGRectangleRasterizationTest, areaNotAligned) {
  // redundant, added for completeness
  test_area_alignement(Directions{});
}

TEST_F(RSGRectangleRasterizationTest, areaAlignesLeft) {
  test_area_alignement(Directions{}.set_left());
}

TEST_F(RSGRectangleRasterizationTest, areaAlignesRight) {
  test_area_alignement(Directions{}.set_right());
}

TEST_F(RSGRectangleRasterizationTest, areaAlignesRightLeft) {
  test_area_alignement(Directions{}.set_right().set_left());
}

TEST_F(RSGRectangleRasterizationTest, areaAlignesUp) {
  test_area_alignement(Directions{}.set_up());
}

TEST_F(RSGRectangleRasterizationTest, areaAlignesUpLeft) {
  test_area_alignement(Directions{}.set_up().set_left());
}

TEST_F(RSGRectangleRasterizationTest, areaAlignesUpRight) {
  test_area_alignement(Directions{}.set_up().set_right());
}

TEST_F(RSGRectangleRasterizationTest, areaAlignesUpRightLeft) {
  test_area_alignement(Directions{}.set_up().set_right().set_left());
}

TEST_F(RSGRectangleRasterizationTest, areaAlignesDown) {
  test_area_alignement(Directions{}.set_down());
}

TEST_F(RSGRectangleRasterizationTest, areaAlignesDownLeft) {
  test_area_alignement(Directions{}.set_down().set_left());
}

TEST_F(RSGRectangleRasterizationTest, areaAlignesDownRight) {
  test_area_alignement(Directions{}.set_down().set_right());
}

TEST_F(RSGRectangleRasterizationTest, areaAlignesDownRightLeft) {
  test_area_alignement(Directions{}.set_down().set_right().set_left());
}

TEST_F(RSGRectangleRasterizationTest, areaAlignesDownUp) {
  test_area_alignement(Directions{}.set_down().set_up());
}

TEST_F(RSGRectangleRasterizationTest, areaAlignesDownUpLeft) {
  test_area_alignement(Directions{}.set_down().set_up().set_left());
}

TEST_F(RSGRectangleRasterizationTest, areaAlignesDownUpRight) {
  test_area_alignement(Directions{}.set_down().set_up().set_right());
}

TEST_F(RSGRectangleRasterizationTest, areaAlignesDownUpRightLeft) {
  test_area_alignement(Directions{}.set_down().set_up().set_right().set_left());
}

//============================================================================//

class RSGInfinityScalingTest : public ::testing::Test {
protected:
  static_assert(std::numeric_limits<double>::has_infinity);
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
