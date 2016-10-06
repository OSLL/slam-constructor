#ifndef __TINY_GRID_CELLS_H
#define __TINY_GRID_CELLS_H

#include "../core/maps/grid_map.h"

//------------------------------------------------------------------------------
// Base cell

class BaseTinyCell : public GridCell {
public:
  BaseTinyCell(): _prob(0.5) {}
  double value() const override { return _prob; }
  void set_value(const Occupancy &value, double quality) override {
    _prob = (1.0 - quality) * _prob + quality * value.prob_occ;
  }
private:
  double _prob;
};

//------------------------------------------------------------------------------
// Modified cell

class AvgTinyCell : public GridCell {
public:
  AvgTinyCell(): _cnt(0), _n(0) {}
  double value() const override { return _n == 0 ? -1 : _cnt / _n; }
  void set_value (const Occupancy &value, double quality) override {
    _n += 1;
    _cnt += 0.5 + (value.prob_occ - 0.5) * quality;
  }
private:
  double _cnt, _n;
};

#endif
