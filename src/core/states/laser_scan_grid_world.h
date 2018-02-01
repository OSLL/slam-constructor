#ifndef SLAM_CTOR_CORE_LASER_SCAN_GRID_WORLD_H
#define SLAM_CTOR_CORE_LASER_SCAN_GRID_WORLD_H

#include "sensor_data.h"
#include "world.h"

template <typename Map>
class LaserScanGridWorld : public World<TransformedLaserScan, Map> {
public: //types
  using MapType = typename World<TransformedLaserScan, Map>::MapType;
  using ScanType = TransformedLaserScan;
public: // methods

  void handle_sensor_data(ScanType &scan) override {
    this->update_robot_pose(scan.pose_delta);
    handle_observation(scan);

    this->notify_with_pose(this->pose());
    this->notify_with_map(this->map());
  }

  virtual void handle_observation(ScanType &tr_scan) = 0;
};

#endif
