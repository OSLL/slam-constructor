#ifndef __VINY_GRID_CELL_H
#define __VINY_GRID_CELL_H

#include "../core/maps/grid_cell.h"
#include <ostream>

class VinyCell : public GridCell {
public:
  VinyCell(): GridCell{Occupancy{-1, 1}}, _n(0) {}

  // TODO: use CRTP
  virtual std::unique_ptr<GridCell> clone() const {
    return std::make_unique<VinyCell>(*this);
  }

  virtual void operator+=(const AreaOccupancyObservation &aoo) {
    if (!aoo.occupancy.is_valid()) { return; }

    _n += 1;
    double that_p = 0.5 + (aoo.occupancy - 0.5) * aoo.quality;
    occupancy.prob_occ = ((*this) * (_n - 1) + that_p) / _n;
  }

private:
  double _n;
};

class BaseDSBelief {
private:
  // TODO: use enum class
  static const int UNKNOWN = 0b00, EMPTY = 0b01, OCCUPIED = 0b10,
                   CONFLICT = 0b11, NM = 4;
public:
  BaseDSBelief() {}
  BaseDSBelief(const BaseDSBelief&) = default;
  BaseDSBelief& operator=(const BaseDSBelief&) = default;
  BaseDSBelief& operator=(BaseDSBelief&&) = default;

  BaseDSBelief(const AreaOccupancyObservation& aoo) {
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

  BaseDSBelief& operator+=(const BaseDSBelief& rhs) {
    auto result = BaseDSBelief();
    result.reset();

    for (int this_id = 0; this_id < NM; ++this_id) {
      for (int that_id = 0; that_id < NM; ++that_id) {
        result.belief_by_id(this_id | that_id) +=
          belief_by_id(this_id) * rhs.belief_by_id(that_id);
      }
    }

    *this = result;
    return *this;
  }

  explicit operator Occupancy() {
    auto occ = Occupancy{_occupied + 0.5 * _unknown, 1.0};
    return occ;
  }

  void normalize() {
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
  void reset() { _occupied = _empty = _unknown = _conflict = 0; }


  double& belief_by_id(int id) {
    return const_cast<double&>(
             static_cast<const BaseDSBelief*>(this)->belief_by_id(id));
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

std::ostream &operator<<(std::ostream &stream, const BaseDSBelief &belief) {
  return stream << "{o: " << belief.occupied() << ", e: " << belief.empty()  \
                << ", dn: " << belief.unknown() \
                << ", c: " << belief.conflict() << "}";
}

/**/

class VinyDSCell : public GridCell {
public:
  VinyDSCell(): GridCell{Occupancy{-1, 1}} {}

  virtual std::unique_ptr<GridCell> clone() const {
    return std::make_unique<VinyDSCell>(*this);
  }

  virtual void operator+=(const AreaOccupancyObservation &aoo) {
    if (!aoo.occupancy.is_valid()) { return; }

    belief += aoo;
    belief.normalize();
    auto belief_prob = ((Occupancy) belief).prob_occ;

    //double that_p = 0.5 + (aoo.occupancy - 0.5) * aoo.quality;

    occupancy.prob_occ = belief_prob;// +that_p) / 2;
  }

  virtual double discrepancy(const AreaOccupancyObservation &aoo) const {
    auto that_belief = BaseDSBelief{aoo};
    that_belief += belief;
    return that_belief.conflict() + belief.unknown();
  }

private:
  BaseDSBelief belief;
};

#endif
