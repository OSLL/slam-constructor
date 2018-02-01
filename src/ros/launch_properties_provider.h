#ifndef SLAM_CTOR_ROS_LAUNCH_PROPERTIES_PROVIDER
#define SLAM_CTOR_ROS_LAUNCH_PROPERTIES_PROVIDER

#include <string>
#include <ros/ros.h>
#include <cassert>

#include "../utils/properties_providers.h"

class LaunchPropertiesProvider : public PropertiesProvider {
public:

  int get_int(const std::string &id, int dflt) const override {
    return launch_param<int>(id, dflt);
  }

  double get_dbl(const std::string &id, double dflt) const override {
    return launch_param<double>(id, dflt);
  }

  str get_str(const std::string &id, const str &dflt) const override {
    return launch_param<str>(id, dflt);
  }

  unsigned get_uint(const std::string &id, unsigned dflt) const override {
    int v = launch_param<int>(id, dflt);
    assert(0 <= v);
    return v;
  }

  bool get_bool(const std::string &id, bool dflt) const override {
    return launch_param<bool>(id, dflt);
  }

private:

  template <typename T>
  T launch_param(const std::string &id, const T &default_value) const {
    T value;
    ros::param::param<T>("~" + id, value, default_value);
    return value;
  }
};

#endif
