#ifndef __GRID_CELL_STRATEGY_H
#define __GRID_CELL_STRATEGY_H

#include <memory>

#include "grid_cell.h"
#include "../grid_scan_matcher.h"
#include "cell_occupancy_estimator.h"

class GridCellStrategy {
public:
  GridCellStrategy(std::shared_ptr<GridCell> prototype,
                   std::shared_ptr<ScanCostEstimator> cost_est,
                   std::shared_ptr<CellOccupancyEstimator> occ_est)
    : _cell_prototype(prototype), _cost_estimator(cost_est),
      _occupancy_estimator(occ_est) {}

  std::shared_ptr<GridCell> cell_prototype() { return _cell_prototype; }
  std::shared_ptr<ScanCostEstimator> cost_est() { return _cost_estimator; }
  std::shared_ptr<CellOccupancyEstimator> occupancy_est() {
    return _occupancy_estimator;
  }

private:
  // cell creation/update
  std::shared_ptr<GridCell> _cell_prototype;
  // cell comparison (scan evaluation)
  std::shared_ptr<ScanCostEstimator> _cost_estimator;
  // new cell value estimation
  std::shared_ptr<CellOccupancyEstimator> _occupancy_estimator;
};

#endif
