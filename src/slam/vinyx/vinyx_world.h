#ifndef SLAM_CTOR_SLAM_VINYX_WORLD_H
#define SLAM_CTOR_SLAM_VINYX_WORLD_H

/*
 * vinySLAM+ world.
 * Uses RBPF to track multiple world state hypotheses.
 * Each hypothesis is tracked by single-hypothesis vinySLAM.
 *
 * author: hatless.fox
 */

#include "../../core/world.h"
#include "../../core/particle_filter.h"

#include "vinyx_hypothesis.h"
#include "../viny/viny_grid_cell.h"

#include "../../core/scan_matchers/m3rsm_engine.h"
#include "../../core/maps/rescalable_caching_grid_map.h"

class VinyXDSCell : public VinyDSCell {
public:
  std::unique_ptr<GridCell> clone() const override {
    return std::make_unique<VinyXDSCell>(*this);
  }

  double discrepancy(const AreaOccupancyObservation &aoo) const override {
    // TODO: consider normalized discrepancy usage in plain vinySLAM
    return belief().normalized_discrepancy(aoo);
  }
};

// FIXME: code duplication (gmaping_particle_filter.h)

class VinyHypothesisFactory : public ParticleFactory<VinyXHypothesis> {
private:
  using GcsPtr = std::shared_ptr<GridCellStrategy>;
public:
  VinyHypothesisFactory(GcsPtr gcs,
                        std::shared_ptr<GridMapScanAdder> scan_adder,
                        const GridMapParams &gmp,
                        const VinyWorldParams &vwp)
    : _gcs{gcs}, _scan_adder{scan_adder}, _gmp{gmp}, _vwp{vwp} {}

  std::shared_ptr<VinyXHypothesis> create_particle() override {
    return std::make_shared<VinyXHypothesis>(_gcs, _scan_adder, _gmp, _vwp);
  }
private:
  GcsPtr _gcs;
  std::shared_ptr<GridMapScanAdder> _scan_adder;
  const GridMapParams _gmp;
  const VinyWorldParams _vwp;
};

class VinyXWorld : public World<TransformedLaserScan,
                                VinyXHypothesis::MapType> {
public:
  using WorldT = World<TransformedLaserScan, VinyXHypothesis::MapType>;
public: // methods
  VinyXWorld(std::shared_ptr<GridCellStrategy> gcs,
             std::shared_ptr<GridMapScanAdder> scan_adder,
             const GridMapParams &gmp, const VinyWorldParams &vwp,
             unsigned hypoth_nm = 1)
    : _gcs{gcs}
    , _pf{std::make_shared<VinyHypothesisFactory>(gcs, scan_adder, gmp, vwp),
          hypoth_nm} {
    for (auto &p : _pf.particles()) {
      p->sample();
    }
    _pf.heaviest_particle();
  }

  void handle_sensor_data(TransformedLaserScan &scan) override {
    update_robot_pose(scan.pose_delta);
    handle_observation(scan);
    notify_with_pose(pose());
    notify_with_map(map());
  }

  void update_robot_pose(const RobotPoseDelta& delta) override {
    for (auto &world : _pf.particles()) {
      world->update_robot_pose(delta);
    }
    _traversed_since_last_resample += delta.abs();
  }

  const WorldT& world() const override {
    return _pf.heaviest_particle();
  }

  const RobotPose& pose() const override { return world().pose(); }
  const VinyXHypothesis::MapType& map() const override { return world().map(); }

protected:

  void handle_observation(TransformedLaserScan &obs) override {
    detect_peaks(obs);
    for (auto &world : _pf.particles()) { world->handle_sensor_data(obs); }

    // NB: weights are updated during scan update for performance reasons
    _pf.normalize_weights();

    /* std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield); */
    /* std::cout << std::setprecision(4); */
    /* for (auto &particle : _pf.particles()) { */
    /*   std::cout << particle->weight() << " "; */
    /* } */
    /* std::cout << std::endl; */
    try_resample();
  }

private:

  bool try_resample() {
    if (_traversed_since_last_resample.sq_dist() <= 0.5 &&
        std::fabs(_traversed_since_last_resample.theta <= 0.2)) {
      return false;
    }
    bool is_resampled = _pf.try_resample();
    if (!is_resampled) { return false; }
    _traversed_since_last_resample.reset();

    return true;
  }

  void detect_peaks(const TransformedLaserScan &raw_scan) {
    //std::cout << "=== Detect Peaks ===" << std::endl;
    M3RSMEngine engine;
    engine.set_translation_lookup_range(0.4, 0.4);
    engine.set_rotation_lookup_range(0.2, 0.05);
    SafeRescalableMap rescalable_map{map()};
    engine.add_scan_matching_request(_gcs->prob_est(), pose(),
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
      engine.add_match(Match{M3RSMEngine::Rect{best_match.translation_drift.center()}, best_match});
    }
    if (1 < peaks_nm) { std::cout << "PEAKS NM: " << peaks_nm << std::endl; }
  }

private: // fields
  std::shared_ptr<GridCellStrategy> _gcs;
  ParticleFilter<VinyXHypothesis> _pf;
  RobotPoseDelta _traversed_since_last_resample;
};

#endif
