#ifndef SLAM_CTOR_CORE_GRID_SCAN_MATCHER_H
#define SLAM_CTOR_CORE_GRID_SCAN_MATCHER_H

#include <vector>
#include <memory>
#include <algorithm>
#include <limits>

#include "../states/state_data.h"
#include "../states/sensor_data.h"
#include "../states/robot_pose.h"
#include "../maps/occupancy_map.h"
#include "../maps/grid_map.h"

class GridScanMatcherObserver {
public:
  virtual void on_matching_start(const RobotPose &,            /*pose*/
                                 const TransformedLaserScan &, /*scan*/
                                 const GridMap &) {}           /*map*/
  virtual void on_scan_test(const RobotPose &,          /*pose*/
                            const LaserScan2D &,        /*scan*/
                            double) {};                 /*score*/
  virtual void on_pose_update(const RobotPose &,   /*pose*/
                              const LaserScan2D &, /*scan*/
                              double) {};          /*score*/
  virtual void on_matching_end(const RobotPose &,   /*delta*/
                               const LaserScan2D &, /* applied scan */
                               double) {};          /*best_score*/
};

/* Estimates p(observation | map & area) */
class OccupancyObservationProbabilityEstimator {
public:
  // TODO: [API Clean Up] GridMap -> OccupancyMap
  virtual double probability(const AreaOccupancyObservation &aoo,
                             const LightWeightRectangle &area,
                             const GridMap &map) const = 0;
  virtual ~OccupancyObservationProbabilityEstimator() = default;
};

/* Client (e.g. a scan matcher) code example:
 * ...
 * ScanProbabilityEstimator &spe = spe();
 * auto scan = spe.filter_scan(raw_scan, map);
 * auto scan_prob = spe.enstimate_scan_probability(scan, ...);
 * ...
 * NB: the _filter_scan_ call isn't obligatory (depends on spe),
 *     but it may improve _spe_ performance.
 */
class ScanProbabilityEstimator {
public: // types
  struct SPEParams {
    // area in the neighborhood of a scan point that should be analyzed
    LightWeightRectangle sp_analysis_area = {0, 0, 0, 0};
    bool scan_is_prerotated = false;
  };
  using OOPE = std::shared_ptr<OccupancyObservationProbabilityEstimator>;
public:
  ScanProbabilityEstimator(OOPE oope) : _oope{oope} {}

  constexpr static double unknown_probability() {
    return std::numeric_limits<double>::quiet_NaN();
  }

  static bool is_prob_unknown(double probability) {
    return probability == unknown_probability();
  }

  OOPE occupancy_observation_probability_estimator() const { return _oope; }
  void set_oope(OOPE occ_obs_prob_est) { _oope = occ_obs_prob_est; }

  double occupancy_observation_probability(const AreaOccupancyObservation &aoo,
                                           const LightWeightRectangle &area,
                                           const GridMap &map) const {
    return _oope->probability(aoo, area, map);
  }

  virtual LaserScan2D filter_scan(const LaserScan2D &scan, const RobotPose &,
                                  const GridMap &) {
    return scan;
  }

  double estimate_scan_probability(const LaserScan2D &scan,
                                   const RobotPose &pose,
                                   const GridMap &map) const {
    return estimate_scan_probability(scan, pose, map, SPEParams{});
  }

  virtual double estimate_scan_probability(const LaserScan2D &scan,
                                           const RobotPose &pose,
                                           const GridMap &map,
                                           const SPEParams &params) const = 0;

  virtual ~ScanProbabilityEstimator() = default;
private:
  OOPE _oope;
};

class GridScanMatcher {
public:
  using ObsPtr = std::shared_ptr<GridScanMatcherObserver>;
  using SPE = std::shared_ptr<ScanProbabilityEstimator>;
  using SPEParams = ScanProbabilityEstimator::SPEParams;
public:
  GridScanMatcher(SPE estimator,
                  double max_x_error = 0, double max_y_error = 0,
                  double max_th_error = 0)
    : _scan_prob_estimator{estimator}
    , _max_x_error{max_x_error}, _max_y_error{max_y_error}
    , _max_th_error{max_th_error} {}
  virtual ~GridScanMatcher() = default;

  // TODO: fix method's name; add pose_delta to return value
  virtual double process_scan(const TransformedLaserScan &scan,
                              const RobotPose &init_pose,
                              const GridMap &map,
                              RobotPoseDelta &pose_delta) = 0;

  virtual void reset_state() {};

  void subscribe(std::shared_ptr<GridScanMatcherObserver> obs) {
    _observers.push_back(obs);
  }

  void unsubscribe(std::shared_ptr<GridScanMatcherObserver> obs) {
    // TODO: replace with more ideomatic C++
    std::vector<std::weak_ptr<GridScanMatcherObserver>> new_observers;
    for (auto &raw_obs : GridScanMatcher::observers()) {
      auto obs_ptr = raw_obs.lock();
      if (obs_ptr && obs_ptr != obs) {
        new_observers.push_back(raw_obs);
      }
    }
    _observers = new_observers;
  }

  void set_lookup_ranges(double x, double y = 0, double th = 0) {
    _max_x_error = x;
    _max_y_error = y;
    _max_th_error = th;
  }

  LaserScan2D filter_scan(const LaserScan2D &scan, const RobotPose &pose,
                          const GridMap &map) {
    return scan_probability_estimator()->filter_scan(scan, pose, map);
  }

  double scan_probability(const LaserScan2D &scan, const RobotPose &pose,
                          const GridMap &map) const {
    return _scan_prob_estimator->estimate_scan_probability(scan, pose, map);
  }

  double scan_probability(const LaserScan2D &scan, const RobotPose &pose,
                          const GridMap &map, const SPEParams &p) const {
    return _scan_prob_estimator->estimate_scan_probability(scan, pose, map, p);
  }

  SPE scan_probability_estimator() const {
    return _scan_prob_estimator;
  }

  void set_scan_probability_estimator(SPE spe) {
    _scan_prob_estimator = spe;
  }

protected:

  std::vector<std::weak_ptr<GridScanMatcherObserver>> & observers() {
    return _observers;
  }

  void do_for_each_observer(std::function<void(ObsPtr)> op) {
    for (auto &obs : observers()) {
      if (auto obs_ptr = obs.lock()) {
        op(obs_ptr);
      }
    }
  }

  double max_x_error() { return _max_x_error; }
  double max_y_error() { return _max_y_error; }
  double max_th_error() { return _max_th_error; }

private:
  std::vector<std::weak_ptr<GridScanMatcherObserver>> _observers;
  SPE _scan_prob_estimator;
  double _max_x_error = 0, _max_y_error = 0, _max_th_error = 0;
};

#endif
