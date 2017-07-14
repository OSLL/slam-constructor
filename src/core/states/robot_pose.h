#ifndef SLAM_CTOR_CORE_ROBOT_POSE_H_INCLUDED
#define SLAM_CTOR_CORE_ROBOT_POSE_H_INCLUDED

#include <iostream>
#include <memory>
#include "../math_utils.h"
#include "../random_utils.h"
#include "../geometry_utils.h"

class RobotPoseDelta {
public: // methods
  RobotPoseDelta() : RobotPoseDelta(0, 0, 0){}
  constexpr RobotPoseDelta(double d_x, double d_y, double d_th) :
    x(d_x), y(d_y), theta(d_th) {}
  constexpr RobotPoseDelta(const Point2D &offset, double d_th) :
    x(offset.x), y(offset.y), theta(d_th) {}

  RobotPoseDelta& operator+=(const RobotPoseDelta& delta) {
    x += delta.x;
    y += delta.y;
    theta += delta.theta;
    return *this;
  }

  bool operator==(const RobotPoseDelta &rhs) const {
    return are_equal(x, rhs.x) && are_equal(y, rhs.y) &&
           are_equal(theta, rhs.theta);
  }

  RobotPoseDelta operator+(const RobotPoseDelta &rhs) const {
    return {x + rhs.x, y + rhs.y, theta + rhs.theta};
  }

  bool is_abs_less(const RobotPoseDelta& that) const {
    //TODO: use double EQ (in this case this is not strictly required)
    #define LESS_ABS(comp) (std::fabs(comp) < std::fabs(that.comp))
    return LESS_ABS(x) && LESS_ABS(y) && LESS_ABS(theta);
    #undef LESS_ABS
  }
  explicit operator bool() const {
    return !are_equal(x, 0) || !are_equal(y, 0) || !are_equal(theta, 0);
  }

  RobotPoseDelta abs() const {
    return RobotPoseDelta{std::abs(x), std::abs(y), std::abs(theta)};
  }
  double sq_dist() const { return x*x + y*y; }

  void reset() { x = y = theta = 0; }
public: // fields
  double x, y, theta;
};

inline std::ostream& operator<<(std::ostream& os, const RobotPoseDelta& rpd) {
  os << "PoseDelta{ x: " << rpd.x << ", y: " << rpd.y;
  return os << ", th: " << rpd.theta << "}";
}

template <typename RandomEngineT>
class RobotPoseDeltaRV {
public:

  RobotPoseDeltaRV(const RandomVariable1D<RandomEngineT> &x_rv,
                   const RandomVariable1D<RandomEngineT> &y_rv,
                   const RandomVariable1D<RandomEngineT> &th_rv)
    : _x_rv{x_rv.clone()}, _y_rv{y_rv.clone()}, _th_rv{th_rv.clone()} {}

  RobotPoseDeltaRV(const RobotPoseDeltaRV &rpd_rv)
    : _x_rv{rpd_rv._x_rv->clone()}, _y_rv{rpd_rv._y_rv->clone()},
      _th_rv{rpd_rv._th_rv->clone()} {}
  RobotPoseDeltaRV& operator=(const RobotPoseDeltaRV &rpd_rv) {
    RobotPoseDeltaRV tmp{rpd_rv};
    std::swap(_x_rv, tmp._x_rv);
    std::swap(_y_rv, tmp._y_rv);
    std::swap(_th_rv, tmp._th_rv);
    return *this;
  }

  RobotPoseDeltaRV(RobotPoseDeltaRV &&rpd_rv) = default;
  RobotPoseDeltaRV& operator=(RobotPoseDeltaRV &&rpd_rv) = default;
  ~RobotPoseDeltaRV() {}

  RobotPoseDelta sample(RandomEngineT &re) {
    return RobotPoseDelta{_x_rv->sample(re), _y_rv->sample(re),
                          _th_rv->sample(re)};
  }

private:
  std::unique_ptr<RandomVariable1D<RandomEngineT>> _x_rv, _y_rv, _th_rv;
};

class RobotPose {
public: // methods
  RobotPose(double x, double y, double theta) : x(x), y(y), theta(theta) {}
  RobotPose(const RobotPoseDelta& rpd) : RobotPose(rpd.x, rpd.y, rpd.theta) {}

  RobotPose() : RobotPose(0, 0, 0){}
  RobotPose(const RobotPose& that) : RobotPose(that.x, that.y, that.theta){}
  RobotPose(const RobotPose&& that) : RobotPose(that.x, that.y, that.theta){}
  ~RobotPose() {}

  RobotPose& operator=(const RobotPose& that) {
    if (this == &that)
      return *this;

    this->x = that.x;
    this->y = that.y;
    this->theta = that.theta;
    return *this;
  }

  const RobotPoseDelta operator-(const RobotPose& that) const {
    return RobotPoseDelta(x - that.x, y - that.y, theta - that.theta);
  }

  const RobotPose operator+(const RobotPoseDelta& delta) const {
    return RobotPose(x + delta.x, y + delta.y, theta + delta.theta);
  }

  const RobotPose& operator+=(const RobotPoseDelta& delta) {
    // TODO: move update policy to Strategy.
    // TODO: original gMapping adds a nose on udpate (motionmodel.cpp:13)
    x += delta.x;
    y += delta.y;
    theta += delta.theta;
    return *this;
  }

  auto point() const { return Point2D{x, y}; }
public:
  double x, y, theta;
};

inline std::ostream& operator<<(std::ostream& os, const RobotPose& rp) {
  return os << "Pose2D{ x: " << rp.x << ", y: " << rp.y
            << ", th: " << rp.theta << "}";
}

#endif
