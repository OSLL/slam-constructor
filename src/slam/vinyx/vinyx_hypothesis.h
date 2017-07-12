#ifndef SLAM_CTOR_SLAM_VINYX_HYPOTHESIS_H
#define SLAM_CTOR_SLAM_VINYX_HYPOTHESIS_H

#include <random>
#include <memory>

#include "../viny/viny_world.h"
#include "../../core/maps/lazy_tiled_grid_map.h"

class VinyXHypothesis : public Particle
                      , public LaserScanGridWorld<UnboundedLazyTiledGridMap> {
private:
  using GRV1D = GaussianRV1D;
public:
  VinyXHypothesis(std::shared_ptr<GridCellStrategy> gcs,
                  std::shared_ptr<GridMapScanAdder> scan_adder,
                  const GridMapParams &gmp, const VinyWorldParams &vwp)
    : LaserScanGridWorld{gcs, scan_adder, gmp}
    , _params{vwp}
    , _scan_matcher{std::make_shared<VinyScanMatcher>(gcs->prob_est(),
                                                      _params.viny_sm_params)}
    , _scan_is_first{true}
    , _rnd_engine{std::random_device{}()}
    , _pose_guess_rv{GRV1D{0, 0.1}, GRV1D{0, 0.1},
                     GRV1D{deg2rad(0), deg2rad(3)}} {}

protected:

  void handle_observation(TransformedLaserScan &scan) override {
    if (!_scan_is_first) {
      // add "noise" to guess extra cost function peak
      //FIXME: update_robot_pose(_pose_guess_rv.sample(_rnd_engine));
    }

    // FIXME: code duplication (viny_world.h)
    _scan_matcher->reset_state();
    RobotPoseDelta pose_delta;
    auto scan_prob = _scan_matcher->process_scan(scan, pose(),
                                                 map(), pose_delta);
    update_robot_pose(pose_delta);

    scan.quality = pose_delta ? _params.localized_scan_quality :
                                _params.raw_scan_quality;
    LaserScanGridWorld::handle_observation(scan);
    _scan_is_first = false;
    Particle::set_weight(scan_prob * Particle::weight());
  }

private:
  VinyWorldParams _params;
  std::shared_ptr<GridScanMatcher> _scan_matcher;

  bool _scan_is_first;
  std::mt19937 _rnd_engine;
  RobotPoseDeltaRV<std::mt19937> _pose_guess_rv;
};

#endif
