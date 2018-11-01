#ifndef SLAM_CTOR_SLAM_VINY_GRID_CELL_H
#define SLAM_CTOR_SLAM_VINY_GRID_CELL_H

#include "../../core/maps/grid_cell.h"
#include "../../core/maps/transferable_belief_model.h"
#include "TBM_prob_conversion.h"
#include <ostream>

class VinyDSCell : public GridCell {
public:
  VinyDSCell(): GridCell{Occupancy{0.5, 1}} {}
  VinyDSCell(double prob): GridCell{Occupancy{prob, 1}} {}

  std::unique_ptr<GridCell> clone() const override {
    return std::make_unique<VinyDSCell>(*this);
  }

  void operator+=(const AreaOccupancyObservation &aoo) override {
    if (!aoo.occupancy.is_valid()) { return; }

    _belief = conjunctive(_belief, AOO_to_TBM(aoo));
    _belief.normalize_conflict();
    _occupancy = TBM_to_O(_belief);
    _is_unknown = false;
  }

  double discrepancy(const AreaOccupancyObservation &aoo) const override {
    auto that_belief = AOO_to_TBM(aoo);
    auto total_unknown = that_belief.unknown() + _belief.unknown();
    auto d_occ = std::abs(that_belief.occupied() - _belief.occupied());
    auto combined_belief = that_belief;
    combined_belief = conjunctive(combined_belief, _belief);
    /* return combined_belief.conflict() + d_occ + total_unknown; */

    // original "combined.conflict() + d_occ + total_unknown" was replaced
    // with the following rule to put the discrepancy in [0; 1].
    auto unknown = total_unknown / 2.0;
    auto known = 1 - unknown;
    auto known_discrepancy = known * (combined_belief.conflict() + d_occ) / 2.0;
    return unknown / 2 + known_discrepancy;
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
  TBM _belief;
};

#endif
