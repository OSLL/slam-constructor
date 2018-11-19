#ifndef SLAM_CTOR_CORE_LASER_SCAN_GRID_WORLD_H
#define SLAM_CTOR_CORE_LASER_SCAN_GRID_WORLD_H

#include "sensor_data.h"
#include "world.h"
#include "../maps/grid_map.h"

/*
 * TODO: Conceptual Refactoring
 *       Introduce an Observation2PoseTransformer that incapsulates
 *       a transformation strategy (a grid + scan matcher, NN, BoW, etc.).
 *       Draft interface:
 *         * convert(Observation obs, RobotPose hint) -> RobotPose
 *           // performs the conversion on each iteration
 *         * notify_with_correspondence(Observation, RobotPose) -> void
 *           // actualize transformer's state with a new correspondence.
 *       NB: transformer's map means correspondence (~ a hash_map)
 *           world's map means environment state (~ a blueprint)
 */
class LaserScanGridWorld : public World<TransformedLaserScan, GridMap> {
public: //types
  using MapType = GridMap;
  using ScanType = TransformedLaserScan;
public: // methods

  void handle_sensor_data(ScanType &scan) override {
    update_robot_pose(scan.pose_delta);
    handle_observation(scan);

    notify_with_pose(this->pose());
    notify_with_map(this->map());
  }

  virtual void handle_observation(ScanType &tr_scan) = 0;
};

#endif
