#ifndef SLAM_CTOR_CORE_TBM_H
#define SLAM_CTOR_CORE_TBM_H

#include <cassert>
#include <array>

/*------------------ declaration ---------------------*/

//transferable belief model
class TBM {
private:
  //private static belief types
  static enum Belief {
    UNKNOWN = 0,    //00
    EMPTY = 1,      //01
    OCCUPIED = 2,   //10
    CONFLICT = 3,   //11
    MAX_BELIEF = 4
  } _belief_types;

  //private variables
  double _beliefs[MAX_BELIEF];

  //private methods
  void set(Belief belief, double value);
  void set_to_zero();
  void set_to_default();
  double get(Belief belief) const;

public:
  //public constructors
  TBM() { set_to_default(); }
  TBM(const TBM&) = default;
  TBM& operator=(const TBM&) = default;
  TBM& operator=(TBM&&) = default;
  TBM(double unkown, double empty, double occupied, double conflict);
  
  //public methods
  double unknown() const;
  double empty() const;
  double occupied() const;
  double conflict() const;
  void normalize();
  void normalize_conflict();

  // friend functions
  friend TBM conjunctive(const TBM& lhs, const TBM& rhs);
  friend TBM disjunctive(const TBM& lhs, const TBM& rhs);
};


/*------------------ definition ---------------------*/


void TBM::set(Belief belief, double value) {
  assert(belief != MAX_BELIEF);
  _beliefs[belief] = value;
}

void TBM::set_to_zero() {
  set(UNKNOWN, 0.0);
  set(EMPTY, 0.0);
  set(OCCUPIED, 0.0);
  set(CONFLICT, 0.0);
}

void TBM::set_to_default() {
  set(UNKNOWN, 1.0);
  set(EMPTY, 0.0);
  set(OCCUPIED, 0.0);
  set(CONFLICT, 0.0);
}

double TBM::get(Belief belief) const {
  assert(belief != MAX_BELIEF);
  return _beliefs[belief];
}

TBM::TBM(double unkown, double empty, double occupied, double conflict) {
  set(UNKNOWN, unkown);
  set(EMPTY, empty);
  set(OCCUPIED, occupied);
  set(CONFLICT, conflict);
}

double TBM::unknown() const {
  return get(UNKNOWN);
}

double TBM::empty() const {
  return get(EMPTY);
}

double TBM::occupied() const {
  return get(OCCUPIED);
}

double TBM::conflict() const {
  return get(CONFLICT);
}

void TBM::normalize() {
  double tot_weight { unknown() + empty() + occupied() + conflict() };
  if (tot_weight == 0.0) {
    set_to_default();
  } else {
    set(UNKNOWN, unknown() / tot_weight);
    set(EMPTY, empty() / tot_weight);
    set(OCCUPIED, occupied() / tot_weight);
    set(CONFLICT, conflict() / tot_weight);
  }
}

void TBM::normalize_conflict() {
  double weight { unknown() + empty() + occupied() };
  if (weight == 0.0) {
    set_to_default();
  } else {
    set(UNKNOWN, unknown() / weight);
    set(EMPTY, empty() / weight);
    set(OCCUPIED, occupied() / weight);
    set(CONFLICT, 0.0);
  }
}

TBM conjunctive(const TBM& lhs, const TBM& rhs) {
  static const std::size_t max_belief { static_cast<std::size_t>(TBM::MAX_BELIEF) };
  static_assert(max_belief != 0, "static_cast from TBM::MAX_BELIEF to std::size_t fails");

  std::array<double, max_belief> tmp_beliefs;
  tmp_beliefs.fill(0.0);

  for (int this_id = 0; this_id < TBM::MAX_BELIEF; ++this_id) {
    for (int that_id = 0; that_id < TBM::MAX_BELIEF; ++that_id) {
      tmp_beliefs[this_id | that_id] +=
        lhs.get(static_cast<TBM::Belief>(this_id)) * rhs.get(static_cast<TBM::Belief>(that_id));
    }
  }

  TBM tbm(tmp_beliefs.at(0), tmp_beliefs.at(1), tmp_beliefs.at(2), tmp_beliefs.at(3));
  tbm.normalize();
  return tbm;
}

TBM disjunctive(const TBM& lhs, const TBM& rhs) {
  static const std::size_t max_belief { static_cast<std::size_t>(TBM::MAX_BELIEF) };
  static_assert(max_belief != 0, "static_cast from TBM::MAX_BELIEF to std::size_t fails");

  std::array<double, max_belief> tmp_beliefs;
  tmp_beliefs.fill(0.0);

  for (std::size_t this_id = 0; this_id < max_belief; ++this_id) {
    for (std::size_t that_id = 0; that_id < max_belief; ++that_id) {
      tmp_beliefs[this_id & that_id] +=
        lhs.get(static_cast<TBM::Belief>(this_id)) * rhs.get(static_cast<TBM::Belief>(that_id));
    }
  }

  TBM tbm(tmp_beliefs.at(0), tmp_beliefs.at(1), tmp_beliefs.at(2), tmp_beliefs.at(3));
  tbm.normalize();
  return tbm;
}

#endif
