#ifndef SLAM_CTOR_CORE_TBM_GRID_CELLS_H
#define SLAM_CTOR_CORE_TBM_GRID_CELLS_H

#include "grid_cell.h"
#include "transferable_belief_model.h"
#include "../states/state_data.h"

class TbmBaseCell : public GridCell {
public:
  TbmBaseCell(): GridCell{Occupancy{0.5, 1}} {}

  void operator+=(const AreaOccupancyObservation &aoo) override {
    if (!aoo.occupancy.is_valid()) { return; }

    _belief = conjunctive(_belief, aoo2tbm(aoo));
    _belief.normalize_conflict();
    _occupancy = tbm2occ(_belief);
    GridCell::on_update();
  }

  double discrepancy(const AreaOccupancyObservation &aoo) const override {
    auto that_belief = aoo2tbm(aoo);
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
protected:
  virtual Occupancy tbm2occ(const TBM& tbm) const = 0;

  TBM aoo2tbm(const AreaOccupancyObservation& aoo) const {
    if (!aoo.occupancy.is_valid()) { return TBM(); }

    // TODO: consider other conversion schemas
    double prob_occ = aoo.occupancy.prob_occ;
    double est_qual = aoo.occupancy.estimation_quality * aoo.quality;

    double occupied = prob_occ * est_qual;
    double empty = (1 - prob_occ) * est_qual;
    return TBM{1.0 - occupied - empty, empty, occupied, 0.0};
    /*
      if (aoo.is_occupied) {
      _occupied = prob_occ * est_qual;
      _unknown = 1 - _occupied;
      } else {
      _empty = (1 - prob_occ) * est_qual;
      _unknown = 1 - _empty;
      }
    */
  }
protected:
  TBM _belief;
};

class TbmOccConsistentCell : public TbmBaseCell {
public:
  std::unique_ptr<GridCell> clone() const override  {
    return std::make_unique<TbmOccConsistentCell>(*this);
  }

protected:
  // TBM -> Occ is consistent with AOO -> TBM
  Occupancy tbm2occ(const TBM& tbm) const override {
    double qual = tbm.occupied() + tbm.empty();
    double p_occu = tbm.occupied() / qual;
    GridCell::on_update();
    return Occupancy { p_occu, qual };
  }
};

class TbmUnknownEvenOccCell : public TbmBaseCell {
public:
  std::unique_ptr<GridCell> clone() const override  {
    return std::make_unique<TbmUnknownEvenOccCell>(*this);
  }
protected:
  // The unknown tbm component is spread evenly between occ/empty
  // when converted to probability.
  Occupancy tbm2occ(const TBM& tbm) const override {
    return Occupancy{ tbm.occupied() + 0.5 * tbm.unknown(), 1.0 };
  }
};

#endif // include guard
