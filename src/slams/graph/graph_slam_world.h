#ifndef SLAM_CTOR_GRAPH_SLAM_WORLD
#define SLAM_CTOR_GRAPH_SLAM_WORLD

#include <iostream>
#include <memory>
#include <unordered_set>
#include <cmath>
#include "../../core/states/single_state_hypothesis_laser_scan_grid_world.h"
#include "../../core/maps/plain_grid_map.h"

#include "pose_graph_map.h"

class NodeCreationOracle {
public:
  NodeCreationOracle(const RobotPoseDelta& delta_limit) :
    _delta_limit(delta_limit) {}

  bool is_node_creation_required(const RobotPoseDelta& delta) {
    _accum_delta += delta;

    if (_accum_delta.is_abs_less(_delta_limit)) {
      return false;
    }

    _accum_delta.reset();
    return true;
  }
private:
  RobotPoseDelta _accum_delta;
  const RobotPoseDelta _delta_limit;
};

// TODO: remove Gridness
class GraphSlamWorld : public LaserScanGridWorld<PoseGraphMap> {
public: //types
  using MapType = PoseGraphMap;
public:

  // NCO: 0.5 m, 0.5 m, 30 deg
  GraphSlamWorld(const SingleStateHypothesisLSGWProperties &props)
    : _props{props}
    , _last_scan(nullptr), _nco{RobotPoseDelta(0.5, 0.5, 0.5)} {}

  auto estimate_pose_delta(const TransformedLaserScan &last_scan,
                           const TransformedLaserScan &curr_scan) {
    auto pose_delta = RobotPoseDelta{};
    // plain cell
    auto rasterized_last =
      PlainGridMap{std::make_shared<GridCell>(Occupancy{0, 0}),
                   {1000, 1000, 0.1}};
    _props.gmsa->append_scan(rasterized_last,
                             -curr_scan.pose_delta, last_scan.scan,
                             last_scan.quality, _props.scan_margin);
    _props.gsm->process_scan(curr_scan, {0, 0, 0},
                             rasterized_last, pose_delta);
    return pose_delta;
  }

  void handle_observation(ScanType &tr_scan) override {
    if (!_last_scan) {
      _last_scan.reset(new TransformedLaserScan());
    } else { // pose rifenement
      update_robot_pose(estimate_pose_delta(*_last_scan, tr_scan));
    }

    *_last_scan = tr_scan;
    if (!_nco.is_node_creation_required(tr_scan.pose_delta)) {
      return;
    }

    _pose_graph.add_node(tr_scan, pose(), 0.7);
  }

  virtual const PoseGraphMap& map() const override { return _pose_graph;}

private:
  SingleStateHypothesisLSGWProperties _props;
  std::shared_ptr<TransformedLaserScan> _last_scan;
  NodeCreationOracle _nco;
  PoseGraphMap _pose_graph;
};


#endif
