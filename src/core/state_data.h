#ifndef __STATE_DATA_H
#define __STATE_DATA_H

#include <memory>

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


#include "maps/grid_map.h"

// TODO: try to simplify template params
template <typename ObservationType, typename MapType>
class World {
public:
  // data-in
  virtual void update_robot_pose(const RobotPoseDelta& delta) {
    _pose += delta;
  }

  virtual void handle_observation(ObservationType&) = 0;

  // data-out
  virtual const World<ObservationType, MapType>& world() const { return *this; }
  virtual const RobotPose& pose() const { return _pose; }
  virtual const MapType& map() const = 0;
private:
  RobotPose _pose;
};

class WorldPoseObserver {
public:
  virtual void on_pose_update(const RobotPose &rs) = 0;
};

template<typename MapType>
class WorldMapObserver {
public:
  virtual void on_map_update(const MapType &map) = 0;
};

template<typename MapType>
class WorldObservable {
public:
  void subscribe_map(std::shared_ptr<WorldMapObserver<MapType>> obs) {
    _world_map_observers.push_back(obs);
  }

  void subscribe_pose(std::shared_ptr<WorldPoseObserver> obs) {
    _world_pose_observers.push_back(obs);
  }

protected: // methods

#define NOTIFY_EACH_WOBSERVER(var)                     \
  for (auto &raw_obs : _world_##var##_observers) {     \
    auto obs_ptr = raw_obs.lock();                     \
    if (obs_ptr) {                                     \
      obs_ptr->on_##var##_update(var);                 \
    }                                                  \
  }

  void notify_with_pose(const RobotPose &pose) {
    NOTIFY_EACH_WOBSERVER(pose);
  }

  void notify_with_map(const MapType &map) {
    NOTIFY_EACH_WOBSERVER(map);
  }

#undef NOTIFY_EACH_WOBSERVER

private:
  std::vector<std::weak_ptr<WorldMapObserver<MapType>>> _world_map_observers;
  std::vector<std::weak_ptr<WorldPoseObserver>> _world_pose_observers;
};

#endif
