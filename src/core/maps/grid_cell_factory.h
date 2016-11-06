#ifndef __GRID_CELL_FACTORY_H
#define __GRID_CELL_FACTORY_H

#include <memory>

#include "cell_occupancy_estimator.h"

class GridCellValue {
public:
  GridCellValue(double prob = 0, double qual = 0) : occupancy{prob, qual} {}
  GridCellValue(const GridCellValue&) = default;
  GridCellValue &operator=(const GridCellValue&) = default;
  GridCellValue(GridCellValue&&) = default;
  GridCellValue &operator=(GridCellValue&&) = default;
  virtual ~GridCellValue() = default;

  operator double() const { return occupancy.prob_occ; }
  explicit operator bool() const { return occupancy.prob_occ == 0; }
public:
  Occupancy occupancy;
};

class GridCell {
public:
  GridCell() = default;
  GridCell(const GridCell& gc) = default;
  GridCell& operator=(const GridCell& gc) = default;
  GridCell(GridCell&& gc) = default;
  GridCell& operator=(GridCell&& gc) = default;
  virtual ~GridCell() = default;

  virtual const GridCellValue& value() const = 0;
  virtual void set_value(const GridCellValue &value, double quality = 1.0) = 0;
  virtual std::shared_ptr<GridCell> clone() const {
    return nullptr;
  }
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
