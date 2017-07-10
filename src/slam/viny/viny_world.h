#ifndef SLAM_CTOR_SLAM_VINY_WORLD_H
#define SLAM_CTOR_SLAM_VINY_WORLD_H

#include <memory>

#include "../../core/world.h"
#include "../../core/sensor_data.h"
#include "../../core/laser_scan_grid_world.h"
#include "../../core/maps/grid_cell_strategy.h"
#include "../../core/maps/plain_grid_map.h"

#include "viny_scan_matcher.h"

struct VinyWorldParams {
  double localized_scan_quality, raw_scan_quality;
  const VinySMParams Viny_SM_Params;

  VinyWorldParams(const VinySMParams& viny_sm_params) :
    Viny_SM_Params(viny_sm_params) {}
};

class VinyWorld : public LaserScanGridWorld<UnboundedPlainGridMap> {
public:
  using Point = DiscretePoint2D;
public:

  VinyWorld(std::shared_ptr<GridCellStrategy> gcs,
            ScanAdder scan_adder,
            const VinyWorldParams &params,
            const GridMapParams &map_params)
    : LaserScanGridWorld(gcs, scan_adder, map_params)
    , _params(params)
    , _scan_matcher{std::make_shared<VinyScanMatcher>(gcs->prob_est(),
                                                      params.Viny_SM_Params)} {}

  std::shared_ptr<GridScanMatcher> scan_matcher() override {
    return _scan_matcher;
  }

protected:

  void handle_observation(TransformedLaserScan &scan) override {
    _scan_matcher->reset_state();

    RobotPoseDelta pose_delta;
    _scan_matcher->process_scan(scan, pose(), map(), pose_delta);
    update_robot_pose(pose_delta);

    scan.quality = pose_delta ? _params.localized_scan_quality :
                                _params.raw_scan_quality;
    LaserScanGridWorld::handle_observation(scan);
  }

private:
  const VinyWorldParams _params;
  std::shared_ptr<GridScanMatcher> _scan_matcher;
};

#endif
