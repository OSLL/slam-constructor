#ifndef SLAM_CTOR_SLAM_TINY_WORLD_H
#define SLAM_CTOR_SLAM_TINY_WORLD_H

#include <memory>

#include "../../core/laser_scan_grid_world.h"
#include "../../core/maps/plain_grid_map.h"
#include "../../core/maps/grid_cell.h"

#include "tiny_scan_matcher.h"

struct TinyWorldParams {
  double localized_scan_quality, raw_scan_quality;
  const TinySMParams Tiny_SM_Params;

  TinyWorldParams(const TinySMParams& tiny_sm_params) :
    Tiny_SM_Params(tiny_sm_params) {}
};

class TinyWorld : public LaserScanGridWorld<UnboundedPlainGridMap> {
public:
  using Point = DiscretePoint2D;
public:

  TinyWorld(std::shared_ptr<GridCellStrategy> gcs,
            ScanAdder scan_adder,
            const TinyWorldParams &params,
            const GridMapParams &map_params)
    : LaserScanGridWorld(gcs, scan_adder, map_params)
    , _params(params)
    , _scan_matcher(std::make_shared<TinyScanMatcher>(gcs->prob_est(),
                                                      params.Tiny_SM_Params)) {}

  std::shared_ptr<GridScanMatcher> scan_matcher() override {
    return _scan_matcher;
  }

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
  const TinyWorldParams _params;
  std::shared_ptr<GridScanMatcher> _scan_matcher;
};

#endif
