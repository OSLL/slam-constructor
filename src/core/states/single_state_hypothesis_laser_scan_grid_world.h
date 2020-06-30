#ifndef SLAM_CTOR_CORE_SINGLE_STATE_HYPOTHESIS_LASER_SCAN_GRID_WORLD_H
#define SLAM_CTOR_CORE_SINGLE_STATE_HYPOTHESIS_LASER_SCAN_GRID_WORLD_H

#include <memory>
#include <utility>

#include "../maps/grid_map.h"
#include "../maps/grid_map_scan_adders.h"
#include "../scan_matchers/grid_scan_matcher.h"

#include "laser_scan_grid_world.h"

struct SingleStateHypothesisLSGWProperties {
  double localized_scan_quality = 1.0;
  double raw_scan_quality = 1.0;
  std::size_t scan_margin = 0;

  std::shared_ptr<GridMap> grid_map;
  std::shared_ptr<GridScanMatcher> gsm;
  std::shared_ptr<GridMapScanAdder> gmsa;
};

class SingleStateHypothesisLaserScanGridWorld
  : public LaserScanGridWorld {
public:
  using Properties = SingleStateHypothesisLSGWProperties;
public:
  SingleStateHypothesisLaserScanGridWorld(const Properties &props)
    : _props{props}, _map{_props.grid_map} {}

  // scan matcher access
  auto scan_matcher() { return _props.gsm; }

  void add_sm_observer(std::shared_ptr<GridScanMatcherObserver> obs) {
    auto sm = scan_matcher();
    if (sm) { sm->subscribe(obs); }
  }

  void remove_sm_observer(std::shared_ptr<GridScanMatcherObserver> obs) {
    auto sm = scan_matcher();
    if (sm) { sm->unsubscribe(obs); }
  }

  // scan adder access
  auto scan_adder() { return _props.gmsa; }

  // state access
  const GridMap& map() const override { return *_map; }
  using LaserScanGridWorld::map; // enable non-const map access

  // TODO: return scan prob
  virtual void handle_observation(TransformedLaserScan &tr_scan) {
    auto sm = scan_matcher();
    sm->reset_state();

    auto pose_delta = RobotPoseDelta{};
    auto score = sm->process_scan(tr_scan, pose(), map(), pose_delta);
    update_robot_pose(pose_delta);

    if (score < 0)
      return;

    tr_scan.quality = pose_delta ? _props.localized_scan_quality
                                 : _props.raw_scan_quality;

    scan_adder()->append_scan(map(), pose(), tr_scan.scan,
                              tr_scan.quality, _props.scan_margin);
  }

protected:
  Properties _props;
  std::shared_ptr<GridMap> _map;
};

#endif
