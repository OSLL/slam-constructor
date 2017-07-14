#ifndef SLAM_CTOR_SLAM_VINY_GRID_CELL_H
#define SLAM_CTOR_SLAM_VINY_GRID_CELL_H

#include "../../core/maps/grid_cell.h"
#include <ostream>

class BaseTBM {
private:
  // TODO: use enum class
  static const int UNKNOWN = 0b00, EMPTY = 0b01, OCCUPIED = 0b10,
                   CONFLICT = 0b11, NM = 4;
public:
  BaseTBM() {}
  BaseTBM(const BaseTBM&) = default;
  BaseTBM& operator=(const BaseTBM&) = default;
  BaseTBM& operator=(BaseTBM&&) = default;

  BaseTBM(const AreaOccupancyObservation& aoo) {
    if (!aoo.occupancy.is_valid()) { return; }

    // TODO: consider other conversion schemas
    double prob_occ = aoo.occupancy.prob_occ;
    double est_qual = aoo.occupancy.estimation_quality * aoo.quality;

    _occupied = prob_occ * est_qual;
    _empty = (1 - prob_occ) * est_qual;
    _unknown = 1 - _occupied - _empty;
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

  BaseTBM(double occupied, double empty, double unknown, double conflict)
    : _occupied(occupied), _empty(empty)
    , _unknown(unknown), _conflict(conflict) {}

  BaseTBM& operator+=(const BaseTBM& rhs) {
    auto result = BaseTBM();
    result.reset();

    for (int this_id = 0; this_id < NM; ++this_id) {
      for (int that_id = 0; that_id < NM; ++that_id) {
        result.belief_by_id(this_id | that_id) +=
          belief_by_id(this_id) * rhs.belief_by_id(that_id);
      }
    }

    *this = result;
    normalize();
    return *this;
  }

  BaseTBM& operator-=(const BaseTBM& rhs) {
    auto result = BaseTBM();
    result.reset();

    for (int this_id = 0; this_id < NM; ++this_id) {
      for (int that_id = 0; that_id < NM; ++that_id) {
        result.belief_by_id(this_id & that_id) +=
          belief_by_id(this_id) * rhs.belief_by_id(that_id);
      }
    }

    *this = result;
    normalize();
    return *this;
  }

  double discrepancy(const BaseTBM &that) const {
    double total_unknown = that.unknown() + unknown();
    double d_occ = std::abs(that.occupied() - occupied());
    BaseTBM combined = that;
    combined += *this;
    return combined.conflict() + d_occ + total_unknown;
  }

  double normalized_discrepancy(const BaseTBM &that) const {
    return discrepancy(that) / 3;
  }

  explicit operator Occupancy() {
    auto occ = Occupancy{_occupied + 0.5 * _unknown, 1.0};
    return occ;
  }

  void normalize_conflict() {
    double weight = _occupied + _empty + _unknown;
    _occupied /= weight;
    _empty /= weight;
    _unknown /= weight;
    _conflict = 0;
  }

  double conflict() const { return _conflict; }
  double occupied() const { return _occupied; }
  double empty()    const { return _empty; }
  double unknown()  const { return _unknown; }
private:
  void normalize() {
    double tot_weight = _occupied + _empty + _conflict + _unknown;
    if (tot_weight == 0.0) {
      _unknown = 1.0;
      return;
    }
    _conflict /= tot_weight;
    _occupied /= tot_weight;
    _empty /= tot_weight;
    _unknown /= tot_weight;
  }

  void reset() { _occupied = _empty = _unknown = _conflict = 0; }

  double& belief_by_id(int id) {
    return const_cast<double&>(
             static_cast<const BaseTBM*>(this)->belief_by_id(id));
  }

  const double& belief_by_id(int id) const {
    switch (id) {
    case OCCUPIED: return _occupied;
    case EMPTY: return _empty;
    case UNKNOWN: return _unknown;
    default: return _conflict;
    }
  }
private:
  double _occupied = 0.0;
  double _empty = 0.0;
  double _unknown = 1.0;
  double _conflict = 0.0;
};

/**/

class VinyDSCell : public GridCell {
public:
  VinyDSCell(): GridCell{Occupancy{-1, 1}} {}

  std::unique_ptr<GridCell> clone() const override {
    return std::make_unique<VinyDSCell>(*this);
  }

  void operator+=(const AreaOccupancyObservation &aoo) override {
    if (!aoo.occupancy.is_valid()) { return; }

    _belief += aoo;
    _belief.normalize_conflict();
    _occupancy.prob_occ = Occupancy(_belief).prob_occ;
  }

  double discrepancy(const AreaOccupancyObservation &aoo) const override {
    return _belief.discrepancy(aoo);
  }

  std::vector<char> serialize() const override {
    Serializer s(GridCell::serialize());
    s << _belief.occupied() << _belief.empty()
      << _belief.unknown() << _belief.conflict();
    return s.result();
  }

  std::size_t deserialize(const std::vector<char>& data,
                          std::size_t pos = 0) override {
    Deserializer d(data, GridCell::deserialize(data, pos));
    double o, e, u, c;
    d >> o >> e >> u >> c;
    _belief = BaseTBM(o, e, u, c);
    return d.pos();
  }

  const BaseTBM& belief() const { return _belief; }
private:
  BaseTBM _belief;
};

#endif
