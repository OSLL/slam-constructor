/**
 * \file
 * \brief Defines some classes related to a robot state.
 * There are classes RobotPose, RobotPoseDelta, World, WorldObserver and
 * WorldObservable.
 */

#ifndef __STATE_DATA_H
#define __STATE_DATA_H

#include <memory>

/**
 * \brief Defines a robot position offset in cartesian coordinates and a
 * rotation angle.
 */
class RobotPoseDelta {
public: // methods
  /// Defines the zero robot offset
  RobotPoseDelta() : RobotPoseDelta(0, 0, 0){}
  /**
   * Initializes a robot offset with given parameters.
   * \param x,y,theta The position and the orientation of a robot pose delta.
   */
  RobotPoseDelta(double d_x, double d_y, double d_th) :
    x(d_x), y(d_y), theta(d_th) {}
  /// Increases the offset value using the given one.
  RobotPoseDelta& operator+=(const RobotPoseDelta& delta) {
    x += delta.x;
    y += delta.y;
    theta += delta.theta;
    return *this;
  }

  
  /**
   * Defines whether the current offset is less than the given one.
   * \param that The comparison object.
   */
  bool is_abs_less(const RobotPoseDelta& that) const {
    //TODO: use double EQ (in this case this is not strictly required)
    #define LESS_ABS(comp) (std::fabs(comp) < std::fabs(that.comp))
    return LESS_ABS(x) && LESS_ABS(y) && LESS_ABS(theta);
    #undef LESS_ABS
  }
  /// Defines the rule to convert the PoseDelta to the boolean value.
  explicit operator bool() const {
    return x != 0.0 || y != 0.0 || theta != 0.0;
  }

  /// Calculates the square of the offset distance.
  double sq_dist() const { return x*x + y*y; }

  /// Updates the offset to zero.
  void reset() { x = y = theta = 0; }
public: // fields
  double x, y, theta; ///< The offset values.
};

/**
 * \brief Defines a robot position in cartesian coordinates and an angle of
 * rotation.
 */
class RobotPose {
public: // methods
  /**
   * Initializes a state of a robot with given parameters.
   * \param x,y,theta The position and the orientation of a robot.
   */
  RobotPose(double x, double y, double theta) : x(x), y(y), theta(theta) {}
  /**
   * Initializes a robot position with the pose delta value.
   * \param rpd The offset value.
   */
  RobotPose(const RobotPoseDelta& rpd) : RobotPose(rpd.x, rpd.y, rpd.theta) {}

  /// Sets a robot in (0,0) oriented as zero angle.
  RobotPose() : RobotPose(0, 0, 0){}
  /// The copy constructor.
  RobotPose(const RobotPose& that) : RobotPose(that.x, that.y, that.theta){}
  /// The move constructor.
  RobotPose(const RobotPose&& that) : RobotPose(that.x, that.y, that.theta){}
  ~RobotPose() {}

  /// Updates the current robot position to be equaled to the given one.
  RobotPose& operator=(const RobotPose& that) {
    if (this == &that)
      return *this;

    this->x = that.x;
    this->y = that.y;
    this->theta = that.theta;
    return *this;
  }

  /// Calculates the radius vector from the current pose to the given one.
  const RobotPoseDelta operator-(const RobotPose& that) const {
    return RobotPoseDelta(x - that.x, y - that.y, theta - that.theta);
  }

  /// Defines the new robot position shifted by the given value.
  const RobotPose operator+(const RobotPoseDelta& delta) const {
    return RobotPose(x + delta.x, y + delta.y, theta + delta.theta);
  }

  /// Updates the current robot position shifting it by the given value.
  const RobotPose& operator+=(const RobotPoseDelta& delta) {
    // TODO: move update policy to Strategy.
    // TODO: original gMapping adds a nose on udpate (motionmodel.cpp:13)
    x += delta.x;
    y += delta.y;
    theta += delta.theta;
    return *this;
  }

public:
  double x, y, theta; ///< The position of robot.
};


#include "maps/grid_map.h"

// TODO: try to simplify template params
/**
 * The controller of robot's merged perceptions of an environment.
 */
template <typename ObservationType, typename MapType>
class World {
public:
  // data-in

  /**
   * Shifts the robot position by the given value.
   * \param delta The offset value.
   */
  virtual void update_robot_pose(const RobotPoseDelta& delta) {
    _pose += delta;
  }

  /// Updates a map according to ObservationType data.
  virtual void handle_observation(ObservationType&) = 0;

  // data-out
  /// Returns this world.
  virtual const World<ObservationType, MapType>& world() const { return *this; }
  /// Returns the robot pose.
  virtual const RobotPose& pose() const { return _pose; }
  /// Returns the map.
  virtual const MapType& map() const = 0;
private:
  RobotPose _pose;
};

template<typename MapType>
class WorldObserver {
public:
  virtual void on_pose_update(const RobotPose &rs) = 0;
  virtual void on_map_update(const MapType &map) = 0;
};

template<typename MapType>
class WorldObservable {
public:
  void subscribe(std::shared_ptr<WorldObserver<MapType>> obs) {
    _world_observers.push_back(obs);
  }

protected: // methods

#define NOTIFY_EACH_WOBSERVER(var)                     \
  for (auto &raw_obs : _world_observers) {             \
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
  std::vector<std::weak_ptr<WorldObserver<MapType>>> _world_observers;
};

#endif
