#ifndef SLAM_CTOR_TESTS_MOCK_GRID_CELL_H_INCLUDED
#define SLAM_CTOR_TESTS_MOCK_GRID_CELL_H_INCLUDED

#include "../../src/core/maps/grid_cell.h"

class MockGridCell : public GridCell {
public:
  static constexpr double Default_Occ_Prob = 0.5;
public:
  MockGridCell(double occ_prob = Default_Occ_Prob)
    : GridCell{Occupancy{occ_prob, 0}} {}

  std::unique_ptr<GridCell> clone() const override {
    return std::make_unique<MockGridCell>(*this);
  }

   void operator+=(const AreaOccupancyObservation &aoo) override {
    _occupancy = aoo.occupancy;
  }

  double discrepancy(const AreaOccupancyObservation &aoo) const override {
    return std::abs(_occupancy - aoo.occupancy);
  }
};

// TODO: move to *.cpp file
constexpr double MockGridCell::Default_Occ_Prob;

#endif
