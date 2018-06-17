#ifndef SLAM_CTOR_SLAM_CREDIBILIST_GRID_CELL_H
#define SLAM_CTOR_SLAM_CREDIBILIST_GRID_CELL_H

#include "../../core/maps/grid_cell.h"
#include "../../core/maps/transferable_belief_model.h"
#include "TBM_prob_conversion.h"
#include <ostream>
#include <cassert>

class CredibilistCell : public GridCell {
public:
  //default constructor
  CredibilistCell() : GridCell{Occupancy()} {
    //we really initiate GridCell by refresh_grid_cell()
    refresh_grid_cell();
  }

  std::unique_ptr<GridCell> clone() const override {
    return std::make_unique<CredibilistCell>(*this);
  }

  //update the map using the scan information
  void operator+=(const AreaOccupancyObservation &aoo) override {
    if (!aoo.occupancy.is_valid()) return;
    _belief = conjunctive(_belief, AOO_to_TBM(aoo));
    _belief.normalize_conflict();
    refresh_grid_cell();
  }
  
  double discrepancy(const AreaOccupancyObservation &aoo) const override {
    return 1.0 - score(aoo);
  }
  
  //disjunctive operator
  double score(const AreaOccupancyObservation &aoo) const {
    TBM tbm {AOO_to_TBM(aoo)};
    TBM disjunctive_tbm = disjunctive(tbm, _belief);
    return disjunctive_tbm.occupied();
  }

  std::vector<char> serialize() const override {
    Serializer s(GridCell::serialize());
    s << _belief.unknown() << _belief.empty()
      << _belief.occupied() << _belief.conflict();
    return s.result();
  }

  std::size_t deserialize(const std::vector<char>& data,
                          std::size_t pos = 0) override {
    Deserializer d(data, GridCell::deserialize(data, pos));
    double u, e, o, c;
    d >> u >> e >> o >> c;
    _belief = TBM(u, e, o, c);
    return d.pos();
  }

  const TBM& belief() const { return _belief; }
  
private:
  //private method
  void refresh_grid_cell() {
    _occupancy = TBM_to_O(_belief);
  }
  
  //private varaible
  TBM _belief;
};

#endif
