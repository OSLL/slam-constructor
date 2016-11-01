#ifndef __TINY_SLAM_FASCADE_H
#define __TINY_SLAM_FASCADE_H

#include <memory>

#include "../core/slam_fascade.h"
#include "../core/maps/grid_cell_strategy.h"

#include "tiny_world.h"

class TinySlamFascade :
  public SlamFascade<TransformedLaserScan>,
  public WorldObservable<TinyWorld::MapType> {
public:
  using MapType = TinyWorld::MapType;
private:
  using ScanMatcherObsPtr = std::shared_ptr<GridScanMatcherObserver>;
public: // methods
  // TODO: copy ctor, move ctor, dtor
  TinySlamFascade(std::shared_ptr<GridCellStrategy> gcs,
                  const TinyWorldParams &params):
    _world(new TinyWorld(gcs, params)) {}

  virtual void handle_sensor_data(TransformedLaserScan &scan) override {
    _world->update_robot_pose(scan.pose_delta);
    _world->handle_observation(scan);

    notify_with_pose(_world->pose());
    notify_with_map(_world->map());
  }

  void add_scan_matcher_observer(ScanMatcherObsPtr obs) {
    _world->scan_matcher()->subscribe(obs);
  }

  void remove_scan_matcher_observer(ScanMatcherObsPtr obs) {
    _world->scan_matcher()->subscribe(obs);
  }

private:
  // Position by IMU, used to calculate delta
  std::unique_ptr<TinyWorld> _world;
};

#endif
