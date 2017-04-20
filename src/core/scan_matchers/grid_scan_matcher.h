#ifndef SLAM_CTOR_CORE_GRID_SCAN_MATCHER_H_INCLUDED
#define SLAM_CTOR_CORE_GRID_SCAN_MATCHER_H_INCLUDED

#include <vector>
#include <memory>
#include <algorithm>
#include <limits>

#include "../state_data.h"
#include "../sensor_data.h"
#include "../robot_pose.h"
#include "../maps/grid_map.h"

class GridScanMatcherObserver {
public:
  virtual void on_matching_start(const RobotPose &,           /*pose*/
                                 const TransformedLaserScan &, /*scan*/
                                 const GridMap &) {}    /*map*/
  virtual void on_scan_test(const RobotPose &,           /*pose*/
                            const TransformedLaserScan &, /*scan*/
                            double) {};                  /*score*/
  virtual void on_pose_update(const RobotPose &,            /*pose*/
                              const TransformedLaserScan &,  /*scan*/
                              double) {};                    /*score*/
  virtual void on_matching_end(const RobotPose &, /*delta*/
                               double) {};         /*best_score*/
};

class ScanCostEstimator {
public:
  double estimate_scan_cost(const RobotPose &pose,
                            const TransformedLaserScan &scan,
                            const GridMap &map) {
    return estimate_scan_cost(pose, scan, map,
                              std::numeric_limits<double>::max());
  }

  virtual double estimate_scan_cost(const RobotPose &pose,
                                    const TransformedLaserScan &scan,
                                    const GridMap &map,
                                    double min_cost) = 0;
  virtual ~ScanCostEstimator() = default;
};

class GridScanMatcher {
public:
  GridScanMatcher(std::shared_ptr<ScanCostEstimator> estimator,
                  double max_x_error = 0, double max_y_error = 0,
                  double max_th_error = 0)
    : _cost_estimator{estimator}
    , _max_x_error{max_x_error}, _max_y_error{max_y_error}
    , _max_th_error{max_th_error} {}
  virtual ~GridScanMatcher() = default;

  virtual double process_scan(const RobotPose &init_pose,
                              const TransformedLaserScan &scan,
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

protected:
  std::shared_ptr<ScanCostEstimator> cost_estimator() {
    return _cost_estimator;
  }

  std::vector<std::weak_ptr<GridScanMatcherObserver>> & observers() {
    return _observers;
  }

  double max_x_error() { return _max_x_error; }
  double max_y_error() { return _max_y_error; }
  double max_th_error() { return _max_th_error; }

private:
  std::vector<std::weak_ptr<GridScanMatcherObserver>> _observers;
  std::shared_ptr<ScanCostEstimator> _cost_estimator;
  double _max_x_error = 0, _max_y_error = 0, _max_th_error = 0;
};

#endif
