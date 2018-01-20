#ifndef SLAM_CTOR_CORE_SENSOR_DATA_H
#define SLAM_CTOR_CORE_SENSOR_DATA_H

#include <cassert>
#include <cmath>
#include <memory>
#include <vector>
#include <iostream>

#include "state_data.h"
#include "robot_pose.h"
#include "../geometry_utils.h"

struct ScanPoint2D {
public:
  enum class PointType {Polar, Cartesian};
public: // methods

  static ScanPoint2D make_polar(double range, double angle, bool is_occ) {
    return ScanPoint2D{PointType::Polar, range, angle, is_occ};
  }

  static ScanPoint2D make_cartesian(const Point2D &p, bool is_occ) {
    return ScanPoint2D{PointType::Cartesian, p.x, p.y, is_occ};
  }

  ScanPoint2D(PointType type, double x_or_range, double y_or_angle, bool is_occ)
    : _type{type}, _factor{1.0}, _is_occupied{is_occ} {

    switch (_type) {
    case PointType::Polar:
      _data.polar.range = x_or_range;
      _data.polar.angle = y_or_angle;
      break;
    case PointType::Cartesian:
      _data.cartesian.x = x_or_range;
      _data.cartesian.y = y_or_angle;
      break;
    default:
      assert(0 && "Unknown point type");
    }
  }

  ScanPoint2D(double range = 0, double ang = 0, bool is_occ = true)
    : ScanPoint2D{PointType::Polar, range, ang, is_occ} {}

  double range() const {
    if (_type == PointType::Polar) {
      return _data.polar.range;
    }
    return std::sqrt(std::pow(_data.cartesian.x, 2) +
                     std::pow(_data.cartesian.y, 2));
  }
  double angle() const {
    if (_type == PointType::Polar) {
      return _data.polar.angle;
    }
    return std::atan2(_data.cartesian.y, _data.cartesian.x);
  }
  double x() const {
    if (_type == PointType::Cartesian) {
      return _data.cartesian.x;
    }
    return _data.polar.range * std::cos(_data.polar.angle);
  }
  double y() const {
    if (_type == PointType::Cartesian) {
      return _data.cartesian.y;
    }
    return _data.polar.range * std::sin(_data.polar.angle);
  }
  bool is_occupied() const { return _is_occupied; }

  ScanPoint2D& set_factor(double factor) { _factor = factor; return *this; }
  double factor() const { return _factor; }

  ScanPoint2D to_cartesian(std::shared_ptr<TrigonometryProvider> tp) const {
    auto point = move_origin(0, 0, tp);
    return ScanPoint2D{PointType::Cartesian, point.x, point.y, _is_occupied};
  }

  // NB: a rotation is preset in a given trigonometry provider
  Point2D move_origin(double d_x, double d_y,
                      std::shared_ptr<TrigonometryProvider> tp) const {
    return Point2D{d_x + range() * tp->cos(angle()),
                   d_y + range() * tp->sin(angle())};

  }

  Point2D move_origin(const Point2D &p,
                      std::shared_ptr<TrigonometryProvider> tp) const {
    return move_origin(p.x, p.y, tp);

  }

  Point2D move_origin(double d_x, double d_y, double d_angle) const {
    auto patched = ScanPoint2D{PointType::Polar,
                               range(), angle() + d_angle,
                               is_occupied()};
    return Point2D{patched.x() + d_x, patched.y() + d_y};
  }

  Point2D move_origin(double d_x, double d_y) const {
    return Point2D{x() + d_x, y() + d_y};
  }

  ScanPoint2D to_cartesian(double d_angle = 0, double d_range = 0) const {
    auto patched = ScanPoint2D{PointType::Polar,
                               range() + d_range, angle() + d_angle,
                               is_occupied()};
    return ScanPoint2D{PointType::Cartesian, patched.x(), patched.y(),
                       patched.is_occupied()};
  }

  ScanPoint2D to_polar(double d_x = 0, double d_y = 0) const {
    auto patched = ScanPoint2D{PointType::Cartesian,
                               x() + d_x, y() + d_y,
                               is_occupied()};
    return ScanPoint2D{PointType::Polar, patched.range(), patched.angle(),
                       patched.is_occupied()};
  }

private: // data structs
  union PointData {
    PointData() : cartesian{0, 0} {}
    struct PolarPoint {
      double range, angle;
    } polar;
    Point2D cartesian;
  };

private: // data
  PointType  _type;
  PointData _data;
  double _factor;
  bool _is_occupied;
};

inline std::ostream& operator<<(std::ostream &osm, const ScanPoint2D &sp) {
  osm << "ScanPoint2D{ r: " << sp.range() << "; a: " << sp.angle() << " <-> ";
  return osm << "x: " << sp.x() << ", y: " << sp.y() << "}";
}

struct LaserScan2D {
public:
  using Points = std::vector<ScanPoint2D>;
public:
  const Points& points() const { return _points; }
  Points& points() {
    return const_cast<Points&>(
      static_cast<const LaserScan2D*>(this)->points());
  }

  LaserScan2D to_cartesian(double angle) const {
    LaserScan2D cartsn_scan;
    cartsn_scan.points().reserve(_points.size());
    cartsn_scan.trig_provider = trig_provider;

    cartsn_scan.trig_provider->set_base_angle(angle);
    for (auto &sp : _points) {
      auto cartesian_sp = sp.to_cartesian(cartsn_scan.trig_provider);
      cartsn_scan.points().push_back(cartesian_sp);
    }
    return cartsn_scan;
  }

public:
  // TODO: create simple and effective way
  //       to translate LS to world by a given pose
  //       Move the provider to LaserScan2D.
  std::shared_ptr<TrigonometryProvider> trig_provider;
private:
  Points _points;
};

struct TransformedLaserScan {
  RobotPoseDelta pose_delta;

  LaserScan2D scan;
  double quality; // 0 - low, 1 - fine
};

#endif
