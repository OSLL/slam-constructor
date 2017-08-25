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
    : _n{resolution}, _hist(_n, 0) /* NB: () - ctor, not {} - init list */ {}

  auto reset(const LaserScan2D &scan) {
    std::fill(std::begin(_hist), std::end(_hist), 0);
    using IndType = LaserScan2D::Points::size_type;
    for (IndType i = 1; i < scan.points().size(); ++i) {
      _hist[hist_index(scan.points(), i)]++;
    }
    return *this;
  }

  Storage::value_type operator[](std::size_t i) const { return _hist[i]; }

  Storage::value_type value(const LaserScan2D::Points &pts,
                            LaserScan2D::Points::size_type pt_i) const {
    if (pt_i == 0) {
      return pts.size(); // ~ignore the first point
    }
    return (*this)[hist_index(pts, pt_i)];
  }

  Storage::size_type max_i() const {
    assert(0 < _hist.size());
    auto max_e = std::max_element(_hist.begin(), _hist.end());
    // assume random access iterator.
    return std::distance(_hist.begin(), max_e);
  }

  auto angle_step() const {
    return deg2rad(180) / _n;
  }

protected:
  std::size_t hist_index(const LaserScan2D::Points &pts,
                         LaserScan2D::Points::size_type i) const {
    assert(0 < i && i < pts.size());
    auto &sp1 = pts[i-1], &sp2 = pts[i];
    double d_x = sp1.x() - sp2.x();
    double d_y = sp1.y() - sp2.y();
    double d_d = std::sqrt(d_x*d_x + d_y*d_y);
    double rate = d_x / d_d;
    if (1.0 < std::abs(rate)) {
      std::cout << "[Warning] HWMDSPE: rate is " << rate << std::endl;
      rate = 1.0;
    }
    auto raw_i = std::size_t(std::floor((1 + rate) * _hist.size() / 2));
    return raw_i % _hist.size();
  }

private:
  Storage::size_type _n;
  Storage _hist;
};

#endif
