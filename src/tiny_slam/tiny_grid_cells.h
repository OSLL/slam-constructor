#ifndef __TINY_GRID_CELLS_H
#define __TINY_GRID_CELLS_H

#include "../core/maps/grid_map.h"

//------------------------------------------------------------------------------
// Base cell

class BaseTinyCell : public GridCell {
public:
  BaseTinyCell(): _value(0.5, 1) {}
  const GridCellValue& value() const override { return _value; }
  void set_value(const GridCellValue &new_value, double quality) override {
    double &prob = _value.occupancy.prob_occ;
    prob = (1.0 - quality) * prob + quality * new_value.occupancy;
  }
private:
  GridCellValue _value;
};

//------------------------------------------------------------------------------
// Modified cell

class AvgTinyCell : public GridCell {
public:
  AvgTinyCell(): _value(0, 1), _n(0), _hits(0) {}
  const GridCellValue& value() const override {
    // TODO: _n == 0 case -- invalid occupancy
    _value.occupancy.prob_occ = _n ? _hits / _n : -1;
    return _value;
  }
  void set_value (const GridCellValue &new_value, double quality) override {
    _n += 1;
    _hits += 0.5 + (new_value.occupancy - 0.5) * quality;
  }
private:
  mutable GridCellValue _value;
  double _n, _hits;
};

#endif
