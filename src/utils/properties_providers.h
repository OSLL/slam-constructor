#ifndef SLAM_CTOR_UTILS_PROPERTIES_PROVIDER_H
#define SLAM_CTOR_UTILS_PROPERTIES_PROVIDER_H

#include <string>

class PropertiesProvider {
protected:
  using str = std::string;
public:
  // TODO: use templates?
  virtual int      get_int(const std::string &id, int dflt) const = 0;
  virtual double   get_dbl(const std::string &id, double dflt) const = 0;
  virtual str      get_str(const std::string &id, const str &dflt) const = 0;
  virtual unsigned get_uint(const std::string &id, unsigned dflt) const = 0;
};

#endif
