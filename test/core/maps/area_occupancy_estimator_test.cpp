#include <gtest/gtest.h>

#include "../../../src/core/maps/area_occupancy_estimator.h"

//--------- Subsuits ---------------------//
// === Empty cell, various directions
// === Beam stops inside a cell
// === Out of cell cases
// === Cell touches
// === Edge touches
// === A beam is collinear to an edge
// === A diagonal beam stops at a vertex
// === Beam starts from a target cell
// === Robot is on an edge
//----------------------------------------//

class AreaOccupancyEstimatorTest : public ::testing::Test {
protected: //consts
  static constexpr double Base_Empty_Prob = 0.01;
  static constexpr double Base_Occup_Prob = 0.95;
  static constexpr double Low_Est_Qual = 0.02;
  static constexpr double Unknown_Est_Qual = 0.7;
protected: // methods
  AreaOccupancyEstimatorTest()
    : aoe{Occupancy{Base_Occup_Prob, 1.0}, Occupancy{Base_Empty_Prob, 1.0},
          Low_Est_Qual, Unknown_Est_Qual}
    , cell{-1, 1, -1, 1} {}
protected: // fields
  AreaOccupancyEstimator aoe;
  Rectangle cell;
};

//----------------------------------------//
// === Empty cell, various directions === //

TEST_F(AreaOccupancyEstimatorTest, emptyFwdHorizPierces) {
  auto beam = Segment2D{{-50, 0}, {50, 0}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, 0.5),
            aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, emptyBwdHorizPierces) {
  auto beam = Segment2D{{50, 0}, {-50, 0}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, 0.5),
            aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, emptyFwdVerticPierces) {
  auto beam = Segment2D{{0, -50}, {0, 50}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, 0.5),
            aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, emptyBwdVerticPierces) {
  auto beam = Segment2D{{0, 50}, {0, -50}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, 0.5),
            aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, emptyDlurDiagPierces) {
  auto beam = Segment2D{{-2, -2}, {2, 2}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, 0.5),
            aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, emptyFwdDiagPiercesNearBrVertex) {
  auto beam = Segment2D{{-0.5, 1.5}, {1.5, -0.5}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, 0.125),
            aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, emptyFwdPiercesBlVertexRight) {
  auto beam = Segment2D{{-3, -2}, {3, 1}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, 0.25),
            aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, emptyFwdVerticPiercesNearCellBorder) {
  auto beam = Segment2D{{0.9, -2}, {0.9, 2}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, 0.05),
            aoe.estimate_occupancy(beam, cell, false));
}

//----------------------------------//
// === Beam stops inside a cell === //

TEST_F(AreaOccupancyEstimatorTest, occFwdHorizStopsCenter) {
  auto beam = Segment2D{{-50, 0}, {0, 0}};
  ASSERT_EQ(Occupancy(0.5, 1.0),
            aoe.estimate_occupancy(beam, cell, true));
}

TEST_F(AreaOccupancyEstimatorTest, emptyFwdHorizStopsCenter) {
  // Clarify
  auto beam = Segment2D{{-50, 0}, {0, 0}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, Unknown_Est_Qual),
            aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, occFwdHorizNotReachingCenter) {
  auto beam = Segment2D{{-50, 0}, {0.5, 0}};
  ASSERT_EQ(Occupancy(0.25, 1),
            aoe.estimate_occupancy(beam, cell, true));
}

TEST_F(AreaOccupancyEstimatorTest, emptyFwdHorizNotReachingCenter) {
  auto beam = Segment2D{{-50, 0}, {0.5, 0}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, Unknown_Est_Qual),
            aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, occBwdHorizNotReachingCenter) {
  auto beam = Segment2D{{50, 0}, {0.5, 0}};
  ASSERT_EQ(Occupancy(0.75, 1),
            aoe.estimate_occupancy(beam, cell, true));
}

TEST_F(AreaOccupancyEstimatorTest, emptyBwdHorizNotReachingCenter) {
  auto beam = Segment2D{{50, 0}, {0.5, 0}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, Unknown_Est_Qual),
            aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, occFwdDiagNotReachingCenter) {
  auto beam = Segment2D{{-50, -50}, {0.5, 0.5}};
  ASSERT_EQ(Occupancy(0.125, 1),
            aoe.estimate_occupancy(beam, cell, true));
}

TEST_F(AreaOccupancyEstimatorTest, emptyFwdDiagNotReachingCenter) {
  auto beam = Segment2D{{-50, -50}, {0.5, 0.5}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, Unknown_Est_Qual),
            aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, occBwdDiagNotReachingCenter) {
  auto beam = Segment2D{{50, 50}, {0.5, 0.5}};
  ASSERT_EQ(Occupancy(0.875, 1),
            aoe.estimate_occupancy(beam, cell, true));
}

TEST_F(AreaOccupancyEstimatorTest, occBwdSkewDiagStopsCenter) {
  auto beam = Segment2D{{-2, 4}, {0, 0}};
  ASSERT_EQ(Occupancy(0.5, 1),
            aoe.estimate_occupancy(beam, cell, true));
}

TEST_F(AreaOccupancyEstimatorTest, emptyBwdSkewDialStopsCenter) {
  auto beam = Segment2D{{-2, 4}, {0, 0}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, Unknown_Est_Qual),
            aoe.estimate_occupancy(beam, cell, false));
}

//---------------------------//
// === Out of cell cases === //

TEST_F(AreaOccupancyEstimatorTest, occNotInteractsWithCell) {
  auto beam = Segment2D{{2, 2}, {3, 3}};
  ASSERT_EQ(Occupancy::invalid(), aoe.estimate_occupancy(beam, cell, true));
}

TEST_F(AreaOccupancyEstimatorTest, emptySkewDiagotCrossCell) {
  auto beam = Segment2D{{0, 50}, {-2, 0}};
  ASSERT_EQ(Occupancy::invalid(), aoe.estimate_occupancy(beam, cell, false));
}

//----------------------//
// === Cell touches === //

TEST_F(AreaOccupancyEstimatorTest, occTouchesLeftTopVertex) {
  auto beam = Segment2D{{-2, 0}, {0, 2}};
  ASSERT_EQ(Occupancy::invalid(), aoe.estimate_occupancy(beam, cell, true));
}

TEST_F(AreaOccupancyEstimatorTest, emptyTouchesLeftTopVertex) {
  auto beam = Segment2D{{-2, 0}, {0, 2}};
  ASSERT_EQ(Occupancy::invalid(), aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, occTouchesLeftBotVertex) {
  auto beam = Segment2D{{-2, 0}, {0, -2}};
  ASSERT_EQ(Occupancy::invalid(), aoe.estimate_occupancy(beam, cell, true));
}

TEST_F(AreaOccupancyEstimatorTest, emptyTouchesLeftBotVertex) {
  auto beam = Segment2D{{-2, 0}, {0, -2}};
  ASSERT_EQ(Occupancy::invalid(), aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, occTouchesRightBotVertex) {
  auto beam = Segment2D{{2, 0}, {0, -2}};
  ASSERT_EQ(Occupancy::invalid(), aoe.estimate_occupancy(beam, cell, true));
}

TEST_F(AreaOccupancyEstimatorTest, emptyTouchesRightBotVertex) {
  auto beam = Segment2D{{2, 0}, {0, -2}};
  ASSERT_EQ(Occupancy::invalid(), aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, occTouchesRightTopVertex) {
  auto beam = Segment2D{{2, 0}, {0, 2}};
  ASSERT_EQ(Occupancy::invalid(), aoe.estimate_occupancy(beam, cell, true));;
}

TEST_F(AreaOccupancyEstimatorTest, emptyTouchesRightTopVertex) {
  auto beam = Segment2D{{2, 0}, {0, 2}};
  ASSERT_EQ(Occupancy::invalid(), aoe.estimate_occupancy(beam, cell, false));
}

//----------------------//
// === Edge touches === //

TEST_F(AreaOccupancyEstimatorTest, emptyFwdVerticStopsRearEdgeThroughCenter) {
  auto beam = Segment2D{{0, -4}, {0, 1}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, 0.5),
            aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, occFwdVerticStopsRearEdgeThroughCenter) {
  auto beam = Segment2D{{0, -4}, {0, 1}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, 1.0),
            aoe.estimate_occupancy(beam, cell, true));
}

TEST_F(AreaOccupancyEstimatorTest, emptyFwdVerticStopsFrontEdge) {
  auto beam = Segment2D{{0, -4}, {0, -1}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, Unknown_Est_Qual),
            aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, occFwdVerticStopsFrontEdge) {
  auto beam = Segment2D{{0, -4}, {0, -1}};
  ASSERT_EQ(Occupancy(1, 1), aoe.estimate_occupancy(beam, cell, true));
}


TEST_F(AreaOccupancyEstimatorTest, occBwdVerticStopsFrontEdge) {
  auto beam = Segment2D{{0, 4}, {0, 1}};
  ASSERT_EQ(Occupancy(1, 1), aoe.estimate_occupancy(beam, cell, true));
}

TEST_F(AreaOccupancyEstimatorTest, emptyBwdVerticStopsFrontEdge) {
  // Clarify: Empty_UD_SideAligned_NotReachingCenter
  auto beam = Segment2D{{0, 4}, {0, 1}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, Unknown_Est_Qual),
            aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, emptyBwdVerticStopsRearEdge) {
  auto beam = Segment2D{{0, 4}, {0, -1}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, 0.5),
            aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, occBwdVerticStopsRearEdge) {
  // Clarify
  auto beam = Segment2D{{0, 4}, {0, -1}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, 1.0),
            aoe.estimate_occupancy(beam, cell, true));
}

TEST_F(AreaOccupancyEstimatorTest, emptyBwdHorizStopsRearEdge) {
  auto beam = Segment2D{{4, 0}, {-1, 0}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, 0.5),
            aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, occBwdHorizStopsRearEdge) {
  auto beam = Segment2D{{4, 0}, {-1, 0}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, 1.0),
            aoe.estimate_occupancy(beam, cell, true));
}

TEST_F(AreaOccupancyEstimatorTest, occBwdHorizStopsFrontEdge) {
  auto beam = Segment2D{{4, 0}, {1, 0}};
  ASSERT_EQ(Occupancy(1, 1), aoe.estimate_occupancy(beam, cell, true));
}

TEST_F(AreaOccupancyEstimatorTest, emptyBwhHorizStopsFrontEdge) {
  auto beam = Segment2D{{4, 0}, {1, 0}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, Unknown_Est_Qual),
            aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, occFwdHorizStopsFrontEdge) {
  auto beam = Segment2D{{-4, 0}, {-1, 0}};
  ASSERT_EQ(Occupancy(1, 1), aoe.estimate_occupancy(beam, cell, true));
}

TEST_F(AreaOccupancyEstimatorTest, emptyFwdHorizStopsFrontEdge) {
  auto beam = Segment2D{{-4, 0}, {-1, 0}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, Unknown_Est_Qual),
            aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, emptyFwdHorizStopsRearEdge) {
  auto beam = Segment2D{{-4, 0}, {1, 0}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, 0.5),
            aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, occFwdHorizStopsRearEdge) {
  auto beam = Segment2D{{-4, 0}, {1, 0}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, 1.0),
            aoe.estimate_occupancy(beam, cell, true));
}

//----------------------------------------//
// === A beam is collinear to an edge === //

TEST_F(AreaOccupancyEstimatorTest, emptyFwdVertLiesOnEdge) {
  // Clarify: Empty_DU_PassesSide
  auto beam = Segment2D{{-1, -2}, {-1, 2}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, Low_Est_Qual),
            aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, occFwdVertLiesOnEdge) {
  auto beam = Segment2D{{-1, -2}, {-1, 2}};
  ASSERT_EQ(Occupancy::invalid(), aoe.estimate_occupancy(beam, cell, true));
}

TEST_F(AreaOccupancyEstimatorTest, occFwdVertStopsOnEdge) {
  // Clarify
  auto beam = Segment2D{{-1, -2}, {-1, 0}};
  ASSERT_EQ(Occupancy(0.5, 1),
            aoe.estimate_occupancy(beam, cell, true));
}

TEST_F(AreaOccupancyEstimatorTest, emptyFwdVertStopsOnEdge) {
  // Clarify: ??
  auto beam = Segment2D{{-1, -2}, {-1, 0}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, Unknown_Est_Qual),
            aoe.estimate_occupancy(beam, cell, false));
}

//-------------------------------------------//
// === A diagonal beam stops at a vertex === //

TEST_F(AreaOccupancyEstimatorTest, occDuDiagAlignedStopsAtFrontVertex) {
  auto beam = Segment2D{{2, -2}, {1, -1}};
  ASSERT_EQ(Occupancy(1, 1), aoe.estimate_occupancy(beam, cell, true));
}


TEST_F(AreaOccupancyEstimatorTest, emptyDuDiagAlignedStopsAtFrontVertex) {
  auto beam = Segment2D{{2, -2}, {1, -1}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, Unknown_Est_Qual),
            aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, emptyDuDiagAlignedStopsAtRearVertex) {
  auto beam = Segment2D{{2, -2}, {-1, 1}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, 0.5),
            aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, occDuDiagAlignedStopsAtRearVertex) {
  auto beam = Segment2D{{2, -2}, {-1, 1}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, 0.5),
            aoe.estimate_occupancy(beam, cell, true));
}

//----------------------------------------//
// === Beam starts from a target cell === //

TEST_F(AreaOccupancyEstimatorTest, emptyInsideStopsOut) {
  auto beam = Segment2D{{-0.5, -0.5}, {0, 50}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, Unknown_Est_Qual),
            aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, occInsideStopsOut) {
  auto beam = Segment2D{{-0.5, -0.5}, {0, 50}};
  ASSERT_EQ(Occupancy::invalid(), aoe.estimate_occupancy(beam, cell, true));
}

TEST_F(AreaOccupancyEstimatorTest, emptyInsideStopsBorder) {
  auto beam = Segment2D{{-0.5, -0.5}, {1, 1}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, Unknown_Est_Qual),
            aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, occInsideStopsBorder) {
  auto beam = Segment2D{{-0.5, -0.5}, {1, 1}};
  ASSERT_EQ(Occupancy::invalid(), aoe.estimate_occupancy(beam, cell, true));
}

TEST_F(AreaOccupancyEstimatorTest, occInsideStopsCenter) {
  auto beam = Segment2D{{-0.5, -0.5}, {0, 0}};
  ASSERT_EQ(Occupancy::invalid(), aoe.estimate_occupancy(beam, cell, true));
}

TEST_F(AreaOccupancyEstimatorTest, emptyInsideStopsCenter) {
  auto beam = Segment2D{{-0.5, -0.5}, {0, 0}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, Unknown_Est_Qual),
            aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, SamePlace) {
  auto beam = Segment2D{{0.5, 0.5}, {0.5, 0.5}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, Unknown_Est_Qual),
            aoe.estimate_occupancy(beam, cell, false));
}

//-----------------------------//
// === Robot is on an edge === //

TEST_F(AreaOccupancyEstimatorTest, emptyOnEdgeOut) {
  auto beam = Segment2D{{0, 1}, {1, -2}};
  ASSERT_EQ(Occupancy(Base_Empty_Prob, 1.0 / 3.0),
            aoe.estimate_occupancy(beam, cell, false));
}

TEST_F(AreaOccupancyEstimatorTest, occOnEdgeOut) {
  auto beam = Segment2D{{0, 1}, {1, -2}};
  ASSERT_EQ(Occupancy::invalid(), aoe.estimate_occupancy(beam, cell, true));
}

TEST_F(AreaOccupancyEstimatorTest, occOnEdgeOutNotThrouhCell) {
  auto beam = Segment2D{{0, 1}, {2, 2}};
  ASSERT_EQ(Occupancy::invalid(), aoe.estimate_occupancy(beam, cell, true));
}

TEST_F(AreaOccupancyEstimatorTest, emptyOnEdgeOutNotThrouhCell) {
  auto beam = Segment2D{{0, 1}, {2, 2}};
  ASSERT_EQ(Occupancy::invalid(), aoe.estimate_occupancy(beam, cell, false));
}

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
