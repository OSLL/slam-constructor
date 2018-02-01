#include <gtest/gtest.h>

#include <memory>
#include <limits>

#include "../mock_grid_cell.h"
#include "../../../src/core/maps/plain_grid_map.h"
#include "../../../src/core/scan_matchers/occupancy_observation_probability.h"

// FIXME: [Refactoring] single case per method.

class BaseOOPETest : public ::testing::Test {
public:
  static constexpr double Map_Scale = 1;
  static constexpr double Default_Prob = 0.0;
public:
  BaseOOPETest()
    : map{std::make_shared<MockGridCell>(Default_Prob),
          {100, 100, Map_Scale}} {
    update_map({0, 1}, 0.25); update_map({1, 1}, 0.00);
    update_map({0, 0}, 1.00); update_map({1, 0}, 0.50);
  }
protected:
  using AOO = AreaOccupancyObservation;
  using LWR = LightWeightRectangle;
  using Shift = Point2D;

  void update_map(const UnboundedPlainGridMap::Coord &area_id, double occ) {
    map.update(area_id, AOO{true, {occ, 1}, {0, 0}, 1});
  }

  template <typename OOPE>
  void test_oope(double expected, const Point2D &obstacle,
                 const LWR &range = {0, 0, 0, 0}) const {
    auto actual = OOPE{}.probability(AOO{true, {1, 1}, obstacle, 1},
                                     range.move_center(obstacle), map);
    ASSERT_NEAR(expected, actual, std::numeric_limits<double>::epsilon());
  }

  auto cell_range() const { return LWR{0, Map_Scale, 0, Map_Scale}; }
  auto mid_cell() const { return Shift{Map_Scale / 2, Map_Scale / 2}; }

  template <int i, int j>
  auto mid() const { return mid_cell() + Shift{i * Map_Scale, j * Map_Scale}; }
protected:
  UnboundedPlainGridMap map;
};

/******************************************************************************/
// == ObstacleBasedOOPE tests

class ObstacleBasedOOPETest : public BaseOOPETest {
protected:
  using OOPE = ObstacleBasedOccupancyObservationPE;
};

TEST_F(ObstacleBasedOOPETest, obstacleMid) {
  test_oope<OOPE>(1, mid<0,0>());
}

TEST_F(ObstacleBasedOOPETest, areaNeutrality) {
  test_oope<OOPE>(1, mid<0,0>(), {-1000, 1000, -1000, 1000});
}

TEST_F(ObstacleBasedOOPETest, shiftInsideCellNeutrality) {
  test_oope<OOPE>(1, mid<0,0>());
  test_oope<OOPE>(1, mid<0,0>() + Shift{Map_Scale, Map_Scale} * 0.25);
}

TEST_F(ObstacleBasedOOPETest, shiftNearCell) {
  test_oope<OOPE>(1.00, mid<0,0>());
  test_oope<OOPE>(0.25, mid<0,0>() + Shift{0, Map_Scale});
  test_oope<OOPE>(0.50, mid<0,0>() + Shift{Map_Scale, 0});
  test_oope<OOPE>(0.00, mid<0,0>() + Shift{Map_Scale, Map_Scale});
}

/******************************************************************************/
// == MaxOOPE tests

class MaxOOPETest : public BaseOOPETest {
protected:
  using OOPE = MaxOccupancyObservationPE;
};

TEST_F(MaxOOPETest, obstacleMid) {
  test_oope<OOPE>(1, mid<0,0>());
}

TEST_F(MaxOOPETest, shiftInsideCell) {
  auto range = cell_range().shrink(2);
  test_oope<OOPE>(0.25, mid<0,1>(), range);
  test_oope<OOPE>(1, mid<0,1>() + Shift{0, -Map_Scale} * 0.5, range);
  test_oope<OOPE>(0.25, mid<0,1>() + Shift{Map_Scale, 0} * 0.5, range);
  test_oope<OOPE>(1, mid<0,1>() + Shift{Map_Scale, -Map_Scale} * 0.5, range);
  test_oope<OOPE>(0.5, mid<1,0>() + Shift{0, Map_Scale} * 0.5, range);
}

TEST_F(MaxOOPETest, severalCellCoverNeutralityToSmallShift) {
  auto range = cell_range().shrink(2);
  test_oope<OOPE>(0.25, mid<0,1>(), range);
  test_oope<OOPE>(1, mid<0,1>() + Shift{0, -Map_Scale} * 0.375, range);
  test_oope<OOPE>(0.25, mid<0,1>() + Shift{Map_Scale, 0} * 0.375, range);
  test_oope<OOPE>(1, mid<0,1>() + Shift{Map_Scale, -Map_Scale} * 0.375, range);
}

TEST_F(MaxOOPETest, observationAreaScaling) {
  auto range = cell_range();
  test_oope<OOPE>(Default_Prob, mid<1, -1>(), range.shrink(2));
  test_oope<OOPE>(0.5, mid<1, -1>(), range);
  test_oope<OOPE>(1, mid<1,-1>(), range.shrink(0.5));
}

/******************************************************************************/
// == MeanOOPE tests

class MeanOOPETest : public BaseOOPETest {
protected:
  using OOPE = MeanOccupancyObservationPE;
};

TEST_F(MeanOOPETest, obstacleMid) {
  test_oope<OOPE>(1, mid<0,0>());
}

TEST_F(MeanOOPETest, shiftInsideCell) {
  auto range = cell_range().shrink(2);
  test_oope<OOPE>(0.25, mid<0,1>(), range);
  test_oope<OOPE>(1.25 / 2, mid<0,1>() + Shift{0, -Map_Scale} * 0.5, range);
  test_oope<OOPE>(0.25 / 2, mid<0,1>() + Shift{Map_Scale, 0} * 0.5, range);
  test_oope<OOPE>(1.75 / 4,
                  mid<0,1>() + Shift{Map_Scale, -Map_Scale} * 0.5, range);
}

TEST_F(MeanOOPETest, severalCellCoverNeutralityToSmallShift) {
  auto range = cell_range().shrink(2);
  test_oope<OOPE>(0.25, mid<0,1>(), range);
  // TODO: cmp with no small shift
  test_oope<OOPE>(1.25 / 2, mid<0,1>() + Shift{0, -Map_Scale} * 0.375, range);
  test_oope<OOPE>(0.25 / 2, mid<0,1>() + Shift{Map_Scale, 0} * 0.375, range);
  test_oope<OOPE>(1.75 / 4,
                  mid<0,1>() + Shift{Map_Scale, -Map_Scale} * 0.375, range);
  test_oope<OOPE>(0.5 / 2, mid<1,0>() + Shift{0, Map_Scale} * 0.375, range);
}

TEST_F(MeanOOPETest, observationAreaScaling) {
  auto range = cell_range();
  test_oope<OOPE>(Default_Prob, mid<1, -1>(), range.shrink(2));
  test_oope<OOPE>(0.5 / 4, mid<1, -1>(), range);
  test_oope<OOPE>(1.5 / 9, mid<1,-1>(), range.shrink(0.5));
}

/******************************************************************************/
// == OverlapWeghtedOOPE tests

class OverlapWeghtedOOPETest : public BaseOOPETest {
protected:
  using OOPE = OverlapWeightedOccupancyObservationPE;
};

TEST_F(OverlapWeghtedOOPETest, obstacleMid) {
  test_oope<OOPE>(1, mid<0,0>());
}

TEST_F(OverlapWeghtedOOPETest, shiftInsideCell) {
  auto range = cell_range().shrink(2);
  test_oope<OOPE>(0.25, mid<0,1>(), range);
  test_oope<OOPE>(0.625, mid<0,1>() + Shift{0, -Map_Scale} * 0.5, range);
  test_oope<OOPE>(0.25 / 2, mid<0,1>() + Shift{Map_Scale, 0} * 0.5, range);
  test_oope<OOPE>(1.75 / 4,
                  mid<0,1>() + Shift{Map_Scale, -Map_Scale} * 0.5, range);
  test_oope<OOPE>(0.5 / 2, mid<1,0>() + Shift{0, Map_Scale} * 0.5, range);
}

TEST_F(OverlapWeghtedOOPETest, severalCellCoverSmallShift) {
  auto range = cell_range().shrink(2);
  test_oope<OOPE>(0.25, mid<0,1>(), range);
  test_oope<OOPE>(0.4375, mid<0,1>() + Shift{0, -Map_Scale} * 0.375, range);
  test_oope<OOPE>(0.1875, mid<0,1>() + Shift{Map_Scale, 0} * 0.375, range);
  test_oope<OOPE>(0.25*0.25*0.5 + 0.25 * 0.75 * 1 + 0.75 * 0.75 * 0.25,
                  mid<0,1>() + Shift{Map_Scale, -Map_Scale} * 0.375, range);
}

TEST_F(OverlapWeghtedOOPETest, observationAreaScaling) {
  auto range = cell_range();
  test_oope<OOPE>(0.5*0.25 + 1*0.125 + 0.25*0.0625,
                  mid<1,0>(), range.shrink(0.5));
}

TEST_F(OverlapWeghtedOOPETest, observationAreaScalingSmalShift) {
  auto range = cell_range();
  test_oope<OOPE>((0.5*1 + 1*0.25 + 0.25*0.0625) / 2.25,
                  mid<1,0>(), range.shrink(2.0 / 3.0));
}

/******************************************************************************/
// == OOPEAssumptions tests

class OOPEAssumptionsTest : public BaseOOPETest {};

TEST_F(OOPEAssumptionsTest, pointAreaLeadsToTheSameProbability) {
  auto point_area = LWR{};
  test_oope<ObstacleBasedOccupancyObservationPE>(0.5, mid<1,0>(), point_area);
  test_oope<MaxOccupancyObservationPE>(0.5, mid<1,0>(), point_area);
  test_oope<MeanOccupancyObservationPE>(0.5, mid<1,0>(), point_area);
  test_oope<OverlapWeightedOccupancyObservationPE>(0.5, mid<1,0>(), point_area);
}

/******************************************************************************/

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
