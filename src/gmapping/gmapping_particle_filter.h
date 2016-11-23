#ifndef __GMAPPING_PARTICLE_FILTER_H
#define __GMAPPING_PARTICLE_FILTER_H

#include <memory>
#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>

#include "../core/world.h"
#include "../core/particle_filter.h"
#include "gmapping_world.h"

class GmappingParticleFactory : public ParticleFactory<GmappingWorld> {
private:
  using GcsPtr = std::shared_ptr<GridCellStrategy>;
public:
  GmappingParticleFactory(GcsPtr gcs, const GridMapParams& params,
                          const GMappingParams& gprms)
    : _gcs(gcs), _map_params(params), _gprms(gprms) {}

  virtual std::shared_ptr<GmappingWorld> create_particle() {
    return std::make_shared<GmappingWorld>(_gcs, _map_params, _gprms);
  }
private:
  GcsPtr _gcs;
  GridMapParams _map_params;
  const GMappingParams _gprms;
};

// TODO: add restriction on particle type
class GmappingParticleFilter :
  public World<TransformedLaserScan, GmappingWorld::MapType> {
public:
  using WorldT = World<TransformedLaserScan, GmappingWorld::MapType>;
public: // methods

  GmappingParticleFilter(std::shared_ptr<GridCellStrategy> gcs,
                         const GridMapParams& params,
                         const GMappingParams& gprms, unsigned n = 1):
    _pf(std::make_shared<GmappingParticleFactory>(gcs, params, gprms), n) {

    for (auto &p : _pf.particles()) {
      p->sample();
    }
  }

  void handle_sensor_data(TransformedLaserScan &scan) override {
    for (auto &world : _pf.particles()) {
      world->handle_sensor_data(scan);
    }
    notify_with_pose(pose());
    notify_with_map(map());
  }

  void update_robot_pose(const RobotPoseDelta& delta) override {
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

  const WorldT& world() const override {
    double max_weight = 0.0;
    std::shared_ptr<WorldT> world(nullptr);
    for (auto &p : _pf.particles()) {
      if (p->weight() < max_weight) { continue; }

      max_weight = p->weight();
      world = p;
    }
    return *world;
  }

  const RobotPose& pose() const override { return world().pose(); }
  const GmappingWorld::MapType& map() const override { return world().map(); }

protected:

  void handle_observation(TransformedLaserScan &obs) override {
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

private: // fields
  ParticleFilter<GmappingWorld> _pf;
  RobotPoseDelta _traversed_since_last_resample;
};

#endif // header-guard
