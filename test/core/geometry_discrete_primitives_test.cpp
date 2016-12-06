#include <gtest/gtest.h>

#include "../../src/core/geometry_discrete_primitives.h"

//============================================================================//
//===                          DiscreteSegment2D                           ===//
//============================================================================//

// 7x7 square, lines to each cell on edge, cw

using DPoints = std::vector<DiscretePoint2D>;

TEST(DiscreteSegment2DRasterizationTest, square00) {
  DPoints gen_pts = DiscreteSegment2D{{0, 0}, {0, 3}};
  ASSERT_EQ(DPoints({{0, 0}, {0, 1}, {0, 2}, {0, 3}}), gen_pts);
}

TEST(DiscreteSegment2DRasterizationTest, square02) {
  DPoints gen_pts = DiscreteSegment2D{{0, 0}, {1, 3}};
  ASSERT_EQ(DPoints({{0, 0}, {0, 1}, {1, 2}, {1, 3}}), gen_pts);
}

TEST(DiscreteSegment2DRasterizationTest, square04) {
  DPoints gen_pts = DiscreteSegment2D{{0, 0}, {2, 3}};
  ASSERT_EQ(DPoints({{0, 0}, {1, 1}, {1, 2}, {2, 3}}), gen_pts);
}

TEST(DiscreteSegment2DRasterizationTest, square07) {
  DPoints gen_pts = DiscreteSegment2D{{0, 0}, {3, 3}};
  ASSERT_EQ(DPoints({{0, 0}, {1, 1}, {2, 2}, {3, 3}}), gen_pts);
}

TEST(DiscreteSegment2DRasterizationTest, square10) {
  DPoints gen_pts = DiscreteSegment2D{{0, 0}, {3, 2}};
  ASSERT_EQ(DPoints({{0, 0}, {1, 1}, {2, 1}, {3, 2}}), gen_pts);
}

TEST(DiscreteSegment2DRasterizationTest, square12) {
  DPoints gen_pts = DiscreteSegment2D{{0, 0}, {3, 1}};
  ASSERT_EQ(DPoints({{0, 0}, {1, 0}, {2, 1}, {3, 1}}), gen_pts);
}

TEST(DiscreteSegment2DRasterizationTest, square15) {
  DPoints gen_pts = DiscreteSegment2D{{0, 0}, {3, 0}};
  ASSERT_EQ(DPoints({{0, 0}, {1, 0}, {2, 0}, {3, 0}}), gen_pts);
}

//

TEST(DiscreteSegment2DRasterizationTest, square17) {
  DPoints gen_pts = DiscreteSegment2D{{0, 0}, {3, -1}};
  ASSERT_EQ(DPoints({{0, 0}, {1, 0}, {2, -1}, {3, -1}}), gen_pts);
}

TEST(DiscreteSegment2DRasterizationTest, square19) {
  DPoints gen_pts = DiscreteSegment2D{{0, 0}, {3, -2}};
  ASSERT_EQ(DPoints({{0, 0}, {1, -1}, {2, -1}, {3, -2}}), gen_pts);
}

TEST(DiscreteSegment2DRasterizationTest, square22) {
  DPoints gen_pts = DiscreteSegment2D{{0, 0}, {3, -3}};
  ASSERT_EQ(DPoints({{0, 0}, {1, -1}, {2, -2}, {3, -3}}), gen_pts);
}

TEST(DiscreteSegment2DRasterizationTest, square25) {
  DPoints gen_pts = DiscreteSegment2D{{0, 0}, {2, -3}};
  ASSERT_EQ(DPoints({{0, 0}, {1, -1}, {1, -2}, {2, -3}}), gen_pts);
}

TEST(DiscreteSegment2DRasterizationTest, square28) {
  DPoints gen_pts = DiscreteSegment2D{{0, 0}, {1, -3}};
  ASSERT_EQ(DPoints({{0, 0}, {0, -1}, {1, -2}, {1, -3}}), gen_pts);
}

TEST(DiscreteSegment2DRasterizationTest, square30) {
  DPoints gen_pts = DiscreteSegment2D{{0, 0}, {0, -3}};
  ASSERT_EQ(DPoints({{0, 0}, {0, -1}, {0, -2}, {0, -3}}), gen_pts);
}

TEST(DiscreteSegment2DRasterizationTest, square32) {
  DPoints gen_pts = DiscreteSegment2D{{0, 0}, {-1, -3}};
  ASSERT_EQ(DPoints({{0, 0}, {0, -1}, {-1, -2}, {-1, -3}}), gen_pts);
}

TEST(DiscreteSegment2DRasterizationTest, square34) {
  DPoints gen_pts = DiscreteSegment2D{{0, 0}, {-2, -3}};
  ASSERT_EQ(DPoints({{0, 0}, {-1, -1}, {-1, -2}, {-2, -3}}), gen_pts);
}

TEST(DiscreteSegment2DRasterizationTest, square37) {
  DPoints gen_pts = DiscreteSegment2D{{0, 0}, {-3, -3}};
  ASSERT_EQ(DPoints({{0, 0}, {-1, -1}, {-2, -2}, {-3, -3}}), gen_pts);
}

TEST(DiscreteSegment2DRasterizationTest, square40) {
  DPoints gen_pts = DiscreteSegment2D{{0, 0}, {-3, -2}};
  ASSERT_EQ(DPoints({{0, 0}, {-1, -1}, {-2, -1}, {-3, -2}}), gen_pts);
}

TEST(DiscreteSegment2DRasterizationTest, square42) {
  DPoints gen_pts = DiscreteSegment2D{{0, 0}, {-3, -1}};
  ASSERT_EQ(DPoints({{0, 0}, {-1, 0}, {-2, -1}, {-3, -1}}), gen_pts);
}

//
TEST(DiscreteSegment2DRasterizationTest, square45) {
  DPoints gen_pts = DiscreteSegment2D{{0, 0}, {-3, 0}};
  ASSERT_EQ(DPoints({{0, 0}, {-1, 0}, {-2, 0}, {-3, 0}}), gen_pts);
}

TEST(DiscreteSegment2DRasterizationTest, square48) {
  DPoints gen_pts = DiscreteSegment2D{{0, 0}, {-3, 1}};
  ASSERT_EQ(DPoints({{0, 0}, {-1, 0}, {-2, 1}, {-3, 1}}), gen_pts);
}

TEST(DiscreteSegment2DRasterizationTest, square50) {
  DPoints gen_pts = DiscreteSegment2D{{0, 0}, {-3, 2}};
  ASSERT_EQ(DPoints({{0, 0}, {-1, 1}, {-2, 1}, {-3, 2}}), gen_pts);
}

TEST(DiscreteSegment2DRasterizationTest, square53) {
  DPoints gen_pts = DiscreteSegment2D{{0, 0}, {-3, 3}};
  ASSERT_EQ(DPoints({{0, 0}, {-1, 1}, {-2, 2}, {-3, 3}}), gen_pts);
}

TEST(DiscreteSegment2DRasterizationTest, square56) {
  DPoints gen_pts = DiscreteSegment2D{{0, 0}, {-2, 3}};
  ASSERT_EQ(DPoints({{0, 0}, {-1, 1}, {-1, 2}, {-2, 3}}), gen_pts);
}

TEST(DiscreteSegment2DRasterizationTest, square58) {
  DPoints gen_pts = DiscreteSegment2D{{0, 0}, {-1, 3}};
  ASSERT_EQ(DPoints({{0, 0}, {0, 1}, {-1, 2}, {-1, 3}}), gen_pts);
}

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
