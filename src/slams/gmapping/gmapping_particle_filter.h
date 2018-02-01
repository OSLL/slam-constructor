#ifndef SLAM_CTOR_GMAPPING_PARTICLE_FILTER_H
#define SLAM_CTOR_GMAPPING_PARTICLE_FILTER_H

#include <memory>
#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>

#include "../../core/states/laser_scan_grid_world.h"
#include "../../core/particle_filter.h"
#include "gmapping_world.h"

class GmappingParticleFactory : public ParticleFactory<GmappingWorld> {
public:
  GmappingParticleFactory(const SingleStateHypothesisLSGWProperties &shw_p,
                          const GMappingParams& gprms)
    : _shw_p(shw_p), _gprms(gprms) {}

  std::shared_ptr<GmappingWorld> create_particle() override {
    return std::make_shared<GmappingWorld>(_shw_p, _gprms);
  }
private:
  const SingleStateHypothesisLSGWProperties _shw_p;
  const GMappingParams _gprms;
};

// TODO: add restriction on particle type
class GmappingParticleFilter :
  public LaserScanGridWorld<GmappingWorld::MapType> {
public:
  using MapType = GmappingWorld::MapType;
  using WorldT = LaserScanGridWorld<MapType>;
public: // methods

  GmappingParticleFilter(const SingleStateHypothesisLSGWProperties &shw_p,
                         const GMappingParams& gprms, unsigned n = 1):
    _pf(std::make_shared<GmappingParticleFactory>(shw_p, gprms), n) {

    for (auto &p : _pf.particles()) {
      p->sample();
    }
    _pf.heaviest_particle().mark_master();
  }

  void handle_sensor_data(TransformedLaserScan &scan) override {
    update_robot_pose(scan.pose_delta);
    handle_observation(scan);
    notify_with_pose(pose());
    notify_with_map(map());
  }

  void update_robot_pose(const RobotPoseDelta& delta) override {
    for (auto &world : _pf.particles()) {
      world->update_robot_pose(delta);
    }
    _traversed_since_last_resample += delta.abs();
  }

  const WorldT& world() const override {
    return _pf.heaviest_particle();
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
    try_resample();

    /*
    std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
    std::cout << std::setprecision(4);
    for (auto &p : _pf.particles()) { std::cout << p->weight() << " "; }
    std::cout << std::endl;
    */
  }
private:

  bool try_resample() {
    if (_traversed_since_last_resample.sq_dist() <= 0.5 &&
        std::fabs(_traversed_since_last_resample.theta <= 0.2)) {
      return false;
    }
    bool is_resampled = _pf.try_resample();
    if (!is_resampled) { return false; }
    _traversed_since_last_resample.reset();
    ensure_master_exists();

    return true;
  }
private:

  void ensure_master_exists() {
    auto master_pred = [](std::shared_ptr<GmappingWorld> p) {
      return p->is_master();
    };
    auto prts = _pf.particles();
    bool master_survived = prts.end() !=
      std::find_if(prts.begin(), prts.end(), master_pred);

    if (!master_survived) {
      _pf.heaviest_particle().mark_master();
    }
  }

private: // fields
  ParticleFilter<GmappingWorld> _pf;
  RobotPoseDelta _traversed_since_last_resample;
};

#endif // header-guard
