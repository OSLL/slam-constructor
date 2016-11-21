#ifndef __ROBOT_POSE_H_INCLUDED
#define __ROBOT_POSE_H_INCLUDED

#include "geometry_utils.h"

class RobotPoseDelta {
public: // methods
  RobotPoseDelta() : RobotPoseDelta(0, 0, 0){}
  RobotPoseDelta(double d_x, double d_y, double d_th) :
    x(d_x), y(d_y), theta(d_th) {}
  RobotPoseDelta& operator+=(const RobotPoseDelta& delta) {
    x += delta.x;
    y += delta.y;
    theta += delta.theta;
    return *this;
  }

  bool is_abs_less(const RobotPoseDelta& that) const {
    //TODO: use double EQ (in this case this is not strictly required)
    #define LESS_ABS(comp) (std::fabs(comp) < std::fabs(that.comp))
    return LESS_ABS(x) && LESS_ABS(y) && LESS_ABS(theta);
    #undef LESS_ABS
  }
  explicit operator bool() const {
    return x != 0.0 || y != 0.0 || theta != 0.0;
  }

  double sq_dist() const { return x*x + y*y; }

  void reset() { x = y = theta = 0; }
public: // fields
  double x, y, theta;
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

public:
  double x, y, theta;
};

#endif
