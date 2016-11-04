#ifndef __GMAPPING_SLAM_FASCADE_H
#define __GMAPPING_SLAM_FASCADE_H

#include <cmath>
#include <vector>
#include <memory>

#include "../core/slam_fascade.h"
#include "../core/maps/grid_cell_strategy.h"
#include "gmapping_particle_filter.h"
#include "gmapping_world.h"

class GmappingParticleFactory : public ParticleFactory<GmappingWorld> {
private:
  using GcsPtr = std::shared_ptr<GridCellStrategy>;
public:
  GmappingParticleFactory(GcsPtr gcs) : _gcs(gcs) {}

  virtual std::shared_ptr<GmappingWorld> create_particle() {
    return std::make_shared<GmappingWorld>(_gcs);
  }
private:
  GcsPtr _gcs;
};

class GmappingSlamFascade :
  public SlamFascade<TransformedLaserScan>,
  public WorldObservable<GmappingWorld::MapType> {
public:
  using MapType = GmappingWorld::MapType;
public: // methods
  // TODO: copy ctor, move ctor, dtor
 GmappingSlamFascade(std::shared_ptr<GridCellStrategy> gcs,
                     unsigned particles_nm) :
    _world{new GmappingParticleFilter<TransformedLaserScan>{
      std::make_shared<GmappingParticleFactory>(gcs), particles_nm}} {}

  virtual void handle_sensor_data(TransformedLaserScan &scan) override {
    // prediction step
    // ## update pose according to motion model (TODO: move to policy)
    _world->update_robot_pose(scan.pose_delta);

    //correction step
    _world->handle_observation(scan);
    // updateWeightsTree (?)

    notify_with_pose(_world->pose());
    notify_with_map(_world->map());
  }
private:
  // Position by IMU, used to calculate delta
  std::unique_ptr<World<TransformedLaserScan, GmappingWorld::MapType>> _world;
};

#endif
