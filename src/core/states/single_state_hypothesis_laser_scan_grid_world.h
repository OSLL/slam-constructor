#ifndef SLAM_CTOR_CORE_SINGLE_STATE_HYPOTHESIS_LASER_SCAN_GRID_WORLD_H
#define SLAM_CTOR_CORE_SINGLE_STATE_HYPOTHESIS_LASER_SCAN_GRID_WORLD_H

#include <memory>
#include <utility>

#include "../maps/grid_map.h"
#include "../maps/grid_map_scan_adders.h"

#include "laser_scan_grid_world.h"

struct SingleStateHypothesisLSGWProperties {
#if __GNUC__ < 5
  SingleStateHypothesisLSGWProperties() {}

  SingleStateHypothesisLSGWProperties
    (
      double localized_scan_quality,
      double raw_scan_quality,
      std::size_t scan_margin,
      std::shared_ptr<GridCell> cell_prototype,
      std::shared_ptr<GridScanMatcher> gsm,
      std::shared_ptr<GridMapScanAdder> gmsa,
      GridMapParams map_props
    )
    : localized_scan_quality(localized_scan_quality)
    , raw_scan_quality(raw_scan_quality)
    , scan_margin(scan_margin)
    , cell_prototype(cell_prototype)
    , gsm(gsm), gmsa(gmsa), map_props(map_props) {}
#endif
  double localized_scan_quality = 1.0;
  double raw_scan_quality = 1.0;
  std::size_t scan_margin = 0;

  std::shared_ptr<GridCell> cell_prototype;
  std::shared_ptr<GridScanMatcher> gsm;
  std::shared_ptr<GridMapScanAdder> gmsa;
  GridMapParams map_props;
};

template <typename MapT>
class SingleStateHypothesisLaserScanGridWorld
  : public LaserScanGridWorld<MapT> {
public:
  using MapType = typename LaserScanGridWorld<MapT>::MapType;
  using Properties = SingleStateHypothesisLSGWProperties;
public:
  SingleStateHypothesisLaserScanGridWorld(const Properties &props)
    : _props{props}
    , _map{_props.cell_prototype->clone(), _props.map_props} {}

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
  const MapType& map() const override { return _map; }
  using LaserScanGridWorld<MapT>::map; // enable non-const map access

  // TODO: return scan prob
  virtual void handle_observation(TransformedLaserScan &tr_scan) {
    auto sm = scan_matcher();
    sm->reset_state();

    auto pose_delta = RobotPoseDelta{};
    sm->process_scan(tr_scan, this->pose(), this->map(), pose_delta);
    this->update_robot_pose(pose_delta);

    tr_scan.quality = pose_delta ? _props.localized_scan_quality
                                 : _props.raw_scan_quality;

    scan_adder()->append_scan(_map, this->pose(), tr_scan.scan,
                              tr_scan.quality, _props.scan_margin);
  }

protected:
  Properties _props;
  MapType _map;
};

#endif
