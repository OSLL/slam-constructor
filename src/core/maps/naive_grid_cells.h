#ifndef SLAM_CTOR_CORE_NAIVE_GRID_CELLS_H
#define SLAM_CTOR_CORE_NAIVE_GRID_CELLS_H

#include "grid_cell.h"

class AffineQualityMergeCell : public GridCell {
public:
  AffineQualityMergeCell(): GridCell{Occupancy{0.5, 1}} {}

  std::unique_ptr<GridCell> clone() const override {
    return std::make_unique<AffineQualityMergeCell>(*this);
  }

  virtual void operator+=(const AreaOccupancyObservation &aoo) {
    if (!aoo.occupancy.is_valid()) { return; }

    const double q = aoo.quality;
    _occupancy.prob_occ = (1.0 - q) * (*this) + q * aoo.occupancy;
    GridCell::on_update();
  }
};

//------------------------------------------------------------------------------

class MeanProbabilityCell : public GridCell {
public:
  MeanProbabilityCell(): GridCell{Occupancy{0.5, 1}}, _n(0) {}

  std::unique_ptr<GridCell> clone() const override {
    return std::make_unique<MeanProbabilityCell>(*this);
  }

  virtual void operator+=(const AreaOccupancyObservation &aoo) {
    if (!aoo.occupancy.is_valid()) { return; }

    _n += 1;
    double that_p = 0.5 + (aoo.occupancy - 0.5) * aoo.quality;
    _occupancy.prob_occ = ((*this) * (_n - 1) + that_p) / _n;
    GridCell::on_update();
  }

private:
  double _n;
};

#endif
