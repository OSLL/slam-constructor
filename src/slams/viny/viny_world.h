#ifndef SLAM_CTOR_SLAM_VINY_WORLD_H
#define SLAM_CTOR_SLAM_VINY_WORLD_H

#include <memory>

#include "../../core/states/world.h"
#include "../../core/states/sensor_data.h"
#include "../../core/states/laser_scan_grid_world.h"
#include "../../core/maps/grid_cell_strategy.h"
#include "../../core/maps/plain_grid_map.h"

#include "viny_scan_matcher.h"

struct VinyWorldParams {
  double localized_scan_quality, raw_scan_quality;
  VinySMParams viny_sm_params;

  VinyWorldParams(const VinySMParams& viny_sm_params)
    : viny_sm_params(viny_sm_params) {}
};

template <typename MapT = UnboundedPlainGridMap>
class VinyWorld : public LaserScanGridWorld<MapT> {
public:
  using Point = DiscretePoint2D;
public:

  VinyWorld(std::shared_ptr<GridCellStrategy> gcs,
            std::shared_ptr<GridMapScanAdder> scan_adder,
            const VinyWorldParams &params,
            const GridMapParams &map_params)
    : LaserScanGridWorld<MapT>::LaserScanGridWorld{gcs, scan_adder, map_params}
    , _params{params}
    , _scan_matcher{std::make_shared<VinyScanMatcher>(gcs->prob_est(),
                                                      params.viny_sm_params)} {}

  std::shared_ptr<GridScanMatcher> scan_matcher() override {
    return _scan_matcher;
  }

  void handle_observation(TransformedLaserScan &scan) override {
    _scan_matcher->reset_state();

    RobotPoseDelta pose_delta;
    _scan_matcher->process_scan(scan, this->pose(), this->map(), pose_delta);
    this->update_robot_pose(pose_delta);

    scan.quality = pose_delta ? _params.localized_scan_quality :
                                _params.raw_scan_quality;
    LaserScanGridWorld<MapT>::handle_observation(scan);
  }

private:
  const VinyWorldParams _params;
  std::shared_ptr<GridScanMatcher> _scan_matcher;
};

#endif
