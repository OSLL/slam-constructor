#ifndef SLAM_CTOR_CORE_LASER_SCAN_GRID_WORLD_H
#define SLAM_CTOR_CORE_LASER_SCAN_GRID_WORLD_H

#include <memory>
#include <utility>

#include "sensor_data.h"
#include "world.h"
#include "../maps/grid_cell_strategy.h"
#include "../maps/grid_map.h"
#include "../maps/grid_map_scan_adders.h"

template <typename Map>
class LaserScanGridWorld : public World<TransformedLaserScan, Map> {
public: //types
  using MapType = Map;
  using ScanType = TransformedLaserScan;
  using DPoint = DiscretePoint2D;
  using ScanMatcherObsPtr = std::shared_ptr<GridScanMatcherObserver>;
  using ScanAdder = std::shared_ptr<GridMapScanAdder>;
public: // methods

  LaserScanGridWorld(std::shared_ptr<GridCellStrategy> gcs,
                     ScanAdder scan_adder, const GridMapParams& params,
                     size_t scan_margin = 0)
    : _map(gcs->cell_prototype(), params), _adder{scan_adder}
    , _scan_margin(scan_margin) {}

  virtual std::shared_ptr<GridScanMatcher> scan_matcher() { return nullptr; }

  void handle_sensor_data(TransformedLaserScan &scan) override {
    this->update_robot_pose(scan.pose_delta);
    handle_observation(scan);

    this->notify_with_pose(this->pose());
    this->notify_with_map(map());
  }

  void add_scan_matcher_observer(ScanMatcherObsPtr obs) {
    auto sm = scan_matcher();
    if (sm) { sm->subscribe(obs); }
  }

  void remove_scan_matcher_observer(ScanMatcherObsPtr obs) {
    auto sm = scan_matcher();
    if (sm) { sm->unsubscribe(obs); }
  }

  virtual const MapType& map() const { return _map; }
  virtual MapType& map() { return _map; }

protected:

  virtual void handle_observation(TransformedLaserScan &tr_scan) {
    const RobotPose& pose = World<TransformedLaserScan, MapType>::pose();
    _adder->append_scan(_map, pose, tr_scan.scan,
                        tr_scan.quality, _scan_margin);
  }

private: // fields
  MapType _map;
  ScanAdder _adder;
  size_t _scan_margin;
};

#endif
