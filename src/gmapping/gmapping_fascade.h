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
    return std::shared_ptr<GmappingWorld>(new GmappingWorld(_gcs));
  }
private:
  GcsPtr _gcs;
};

class GmappingSlamFascade :
  public SlamFascade<TransformedLaserScan>,
  public WorldObservable<GmappingWorld::MapType> {
private:
  using PFctrPtr = std::shared_ptr<ParticleFactory<GmappingWorld>>;
public: // methods
  // TODO: copy ctor, move ctor, dtor
  GmappingSlamFascade(std::shared_ptr<GridCellStrategy> gcs) :
    _world(new GmappingParticleFilter<TransformedLaserScan>(
      PFctrPtr(new GmappingParticleFactory(gcs)), 1)) {}

  virtual void handle_sensor_data(TransformedLaserScan &scan) override {
    // prediction step
    // ## update pose according to motion model (TODO: move to policy)
    _world->update_robot_pose(scan.pose_delta);
    _pose_delta += scan.pose_delta;

    // TODO: add threashold to speedup
    //double d_dist = std::pow(_d_x, 2) + std::pow(_d_y, 2);
    //if (std::isnan(_d_yaw) || 1 < d_dist || 0.5 < std::fabs(_d_yaw)) {
      //correction step
    _world->handle_observation(scan);
    //  _d_x = _d_y = _d_yaw = 0;
    //}

    // updateWeightsTree (?)

    notify_with_pose(_world->pose());
    notify_with_map(_world->map());
  }
private:
  RobotPoseDelta _pose_delta;
  // Position by IMU, used to calculate delta
  std::unique_ptr<World<TransformedLaserScan, GmappingWorld::MapType>> _world;
};

#endif
