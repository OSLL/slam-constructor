#ifndef SLAM_CTOR_FEATURES_ANGLE_HISTOGRAM_H
#define SLAM_CTOR_FEATURES_ANGLE_HISTOGRAM_H

#include <vector>
#include <algorithm>

#include "../states/sensor_data.h"

class AngleHistogram {
private:
  using Storage = std::vector<unsigned>;
public:
  AngleHistogram(Storage::size_type resolution = 20)
    : _n{resolution}, _hist(_n, 0) /* NB: () - ctor, not {} - init list */
    , _ang_sum(_n, 0.0) {}

  auto reset(const LaserScan2D &scan) {
    std::fill(std::begin(_hist), std::end(_hist), 0);
    std::fill(std::begin(_ang_sum), std::end(_ang_sum), 0.0);

    const auto &pts = scan.points();
    _drift_dirs.resize(pts.size());

    using IndType = LaserScan2D::Points::size_type;
    for (IndType i = 1; i < pts.size(); ++i) {
      auto angle = estimate_ox_based_angle(pts[i-1], pts[i]);
      auto hist_i = hist_index(angle);
      _hist[hist_i]++;
      _ang_sum[hist_i] += angle;
      _drift_dirs[i] = angle;
    }
    return *this;
  }

  Storage::value_type operator[](std::size_t i) const { return _hist[i]; }

  Storage::value_type value(const LaserScan2D::Points &pts,
                            LaserScan2D::Points::size_type pt_i) const {
    if (pt_i == 0) {
      return pts.size(); // ~ignore the first point
    }
    return (*this)[hist_index(_drift_dirs[pt_i])];
  }

  Storage::size_type max_i() const {
    assert(0 < _hist.size());
    auto max_e = std::max_element(_hist.begin(), _hist.end());
    // assume random access iterator.
    return std::distance(_hist.begin(), max_e);
  }

  Storage::size_type min_i() const {
    assert(0 < _hist.size());
    auto max_e = std::min_element(_hist.begin(), _hist.end());
    // assume random access iterator.
    return std::distance(_hist.begin(), max_e);
  }

  auto least_freq_angle() const {
    auto i = min_i();
    return _ang_sum[i] / _hist[i];
  }

  double angle_step() const {
    return deg2rad(180) / _n;
  }

  // TODO: better naming
  static double estimate_ox_based_angle(
    const LaserScan2D::Points::value_type &base,
    const LaserScan2D::Points::value_type &sp) {

    auto d_x = sp.x() - base.x();
    auto d_y = sp.y() - base.y();
    if (d_y == 0) { // TODO: math utils
      return 0; // 180 is equivalent to 0
    }
    auto d_d = std::sqrt(d_x*d_x + d_y*d_y);
    auto angle = std::acos(d_x / d_d);

    if (d_y < 0 && d_x != 0) { // TODO: math utils
      angle = M_PI - angle;
    }

    return angle;
  }

protected:
  std::size_t hist_index(double angle) const {
    assert(0 <= angle && angle < M_PI);
    auto hist_i = std::size_t(std::floor(angle / angle_step()));
    assert(hist_i < _hist.size());
    return hist_i;
  }

private:
  Storage::size_type _n;
  Storage _hist;
  std::vector<double> _ang_sum, _drift_dirs;
};

#endif
