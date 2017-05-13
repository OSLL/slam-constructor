#ifndef SLAM_CTOR_CORE_GRID_CELL_STRATEGY_H_INCLUDED
#define SLAM_CTOR_CORE_GRID_CELL_STRATEGY_H_INCLUDED

#include <memory>

#include "grid_cell.h"
#include "../scan_matchers/grid_scan_matcher.h"
#include "cell_occupancy_estimator.h"

class GridCellStrategy {
public:
  GridCellStrategy(std::shared_ptr<GridCell> prototype,
                   std::shared_ptr<ScanProbabilityEstimator> prob_est,
                   std::shared_ptr<CellOccupancyEstimator> occ_est)
    : _cell_prototype(prototype), _prob_estimator(prob_est),
      _occupancy_estimator(occ_est) {}

  std::shared_ptr<GridCell> cell_prototype() { return _cell_prototype; }
  std::shared_ptr<ScanProbabilityEstimator> prob_est() {
    return _prob_estimator;
  }
  std::shared_ptr<CellOccupancyEstimator> occupancy_est() {
    return _occupancy_estimator;
  }

private:
  // cell creation/update
  std::shared_ptr<GridCell> _cell_prototype;
  // cell comparison (scan evaluation)
  std::shared_ptr<ScanProbabilityEstimator> _prob_estimator;
  // new cell value estimation
  std::shared_ptr<CellOccupancyEstimator> _occupancy_estimator;
};

#endif
