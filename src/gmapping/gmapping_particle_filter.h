#ifndef __GMAPPING_PARTICLE_FILTER_H
#define __GMAPPING_PARTICLE_FILTER_H

#include <memory>
#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>

#include "../core/state_data.h"
#include "../core/particle_filter.h"
#include "gmapping_world.h"

// TODO: add restriction on particle type
template <typename ObservationType>
class GmappingParticleFilter :
  public World<ObservationType, GmappingWorld::MapType> {
public:
  using WorldT = World<ObservationType, GmappingWorld::MapType>;
public: // methods
  GmappingParticleFilter(
    std::shared_ptr<ParticleFactory<GmappingWorld>> part_fctr,
    unsigned n = 1): _pf(part_fctr, n) {

    for (auto &p : _pf.particles()) {
      p->sample();
    }
  }

  virtual void update_robot_pose(const RobotPoseDelta& delta) override {
    for (auto &world : _pf.particles()) {
      world->update_robot_pose(delta);
    }
    _traversed_since_last_resample += delta;

    if (0.8 < _traversed_since_last_resample.sq_dist() &&
        0.4 < std::fabs(_traversed_since_last_resample.theta) &&
      _pf.try_resample()) {
      _traversed_since_last_resample.reset();
    }
  }

  virtual void handle_observation(ObservationType &obs) override {
    for (auto &world : _pf.particles()) {
      world->handle_observation(obs);
    }

    // NB: weights are updated during scan update for performance reasons
    _pf.normalize_weights();
    /*
    std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
    std::cout << std::setprecision(4);
    for (auto &p : _pf.particles()) { std::cout << p.weight() << " "; }
    std::cout << std::endl;
    */
  }

  virtual const WorldT& world() const {
    double max_weight = 0.0;
    std::shared_ptr<WorldT> world(nullptr);
    for (auto &p : _pf.particles()) {
      if (p->weight() < max_weight) { continue; }

      max_weight = p->weight();
      world = p;
    }
    return *world;
  }

  virtual const RobotPose& pose() const { return world().pose(); }
  virtual const GmappingWorld::MapType& map() const { return world().map(); }
private: // fields
  ParticleFilter<GmappingWorld> _pf;
  RobotPoseDelta _traversed_since_last_resample;
};

#endif // header-guard
