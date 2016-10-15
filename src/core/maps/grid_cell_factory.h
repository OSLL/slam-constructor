#ifndef __GRID_CELL_FACTORY_H
#define __GRID_CELL_FACTORY_H

#include <memory>

#include "cell_occupancy_estimator.h"

class GridCellValue {
public:
  GridCellValue(double prob = 0, double qual = 0) :
    _init_occ{prob, qual}, occupancy{_init_occ} {}
  operator double() const { return occupancy.prob_occ; }
  explicit operator bool() const { return occupancy.prob_occ == 0; }

  virtual void reset() {
    occupancy = _init_occ;
  }

private:
  Occupancy _init_occ;
public:
  Occupancy occupancy;
};

class GridCell {
public:
  virtual const GridCellValue& value() const = 0;
  virtual void set_value(const GridCellValue &value, double quality = 1.0) = 0;
};

class GridCellFactory {
public:
  virtual std::shared_ptr<GridCell> create_cell() = 0;
  virtual std::shared_ptr<GridCellValue> create_cell_value() = 0;
};

template <typename CellType, typename CellValueType=GridCellValue>
class PlainGridCellFactory : public GridCellFactory {
public:
  std::shared_ptr<GridCell> create_cell() override {
    return std::make_shared<CellType>();
  }
  std::shared_ptr<GridCellValue> create_cell_value() override {
    return std::make_shared<CellValueType>();
  }
};

#endif
