#ifndef SLAM_CTOR_SLAM_VINYX_WORLD_H
#define SLAM_CTOR_SLAM_VINYX_WORLD_H

/*
 * vinySLAM+ world.
 * FIXME: add a brief description
 * Each hypothesis is tracked by single-hypothesis vinySLAM.
 *
 * author: hatless.fox
 */

#include <vector>
#include <memory>
#include "../../core/states/single_state_hypothesis_laser_scan_grid_world.h"

#include "../viny/viny_grid_cell.h"

#include "../../core/scan_matchers/m3rsm_engine.h"
#include "../../core/maps/rescalable_caching_grid_map.h"
#include "../../core/maps/lazy_tiled_grid_map.h"

class VinyXDSCell : public VinyDSCell {
public:
  std::unique_ptr<GridCell> clone() const override {
    return std::make_unique<VinyXDSCell>(*this);
  }

  double discrepancy(const AreaOccupancyObservation &aoo) const override {
    // TODO: consider normalized discrepancy usage in plain vinySLAM
    // FIXME:
    return 0;
  }
};

// FIXME: rm from the global namespace
using VinyXMapT = RescalableCachingGridMap<UnboundedLazyTiledGridMap>;

class VinyXWorld : public World<TransformedLaserScan,
                                VinyXMapT> {
public:
  using WorldT = SingleStateHypothesisLaserScanGridWorld<VinyXMapT>;
  using Properties = SingleStateHypothesisLSGWProperties;
public: // methods
  VinyXWorld(const Properties &props)
    : _props{props} {
    _hypotheses.push_back(WorldT{props});
  }

  void handle_sensor_data(TransformedLaserScan &scan) override {
    update_robot_pose(scan.pose_delta);
    handle_observation(scan);
    notify_with_pose(pose());
    notify_with_map(map());
  }

  void update_robot_pose(const RobotPoseDelta& delta) override {
    for (auto &h : _hypotheses) {
      h.update_robot_pose(delta);
    }
  }

  const WorldT& world() const override {
    // TODO: return a max peak?
    return _hypotheses[0];
  }

  const RobotPose& pose() const override { return world().pose(); }
  const VinyXMapT& map() const override { return world().map(); }

  void handle_observation(TransformedLaserScan &obs) override {
    //detect_peaks(obs);
    for (auto &h : _hypotheses) {
      // FIXME: use peaks info in order to just update world state
      h.handle_observation(obs);
    }
  }

private:

  void detect_peaks(const TransformedLaserScan &raw_scan) {
    //std::cout << "=== Detect Peaks ===" << std::endl;
    M3RSMEngine engine;
    engine.set_translation_lookup_range(0.4, 0.4);
    engine.set_rotation_lookup_range(0.2, 0.05);
    SafeRescalableMap rescalable_map{map()};
    // FIXME: use custom SPE, not necessary GridWorldOne
    auto spe = _props.gsm->scan_probability_estimator();
    engine.add_scan_matching_request(spe, pose(),
                                     raw_scan.scan, rescalable_map, true);
    double max_prob = -1;
    unsigned peaks_nm = 0;
    while (1) {
      auto best_match = engine.next_best_match(0.4);
      if (!best_match.is_valid()) {
        break;
      }
      if (best_match.is_finest()) {
        double p = best_match.prob_upper_bound;
        if (max_prob == -1) { max_prob = p; }
        //std::cout << "next prob: " << p << std::endl;
        if (p < max_prob * 0.97 || p < 0.5) {
          break;
        }
        peaks_nm++;
        continue;
      }
      auto drift_center = best_match.translation_drift.center();
      engine.add_match(Match{M3RSMEngine::Rect{drift_center}, best_match});
    }
    if (1 < peaks_nm) { std::cout << "PEAKS NM: " << peaks_nm << std::endl; }
  }

private: // fields
  Properties _props;
  std::vector<WorldT> _hypotheses;
};

#endif
