#ifndef SLAM_CTOR_UTILS_PROPERTIES_PROVIDER_H
#define SLAM_CTOR_UTILS_PROPERTIES_PROVIDER_H

#include <string>
#include <unordered_map>
#include <fstream>
#include <algorithm>

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

class MapPropertiesProvider : public PropertiesProvider {
public:

  void set_property(const std::string &id, const std::string &value) {
    _props[id] = value;
  }

  bool has_property(const std::string &id) const {
    return _props.find(id) != _props.end();
  }

  int get_int(const std::string &id, int dflt) const override {
    return has_property(id) ? std::stoi(_props.at(id)) : dflt;
  }

  double get_dbl(const std::string &id, double dflt) const override {
    return has_property(id) ? std::stod(_props.at(id)) : dflt;
  }

  str get_str(const std::string &id, const str &dflt) const override {
    return has_property(id) ? _props.at(id) : dflt;
  }

  unsigned get_uint(const std::string &id, unsigned dflt) const override {
    return has_property(id) ? std::stoul(_props.at(id)) : dflt;
  }

private:

  std::unordered_map<std::string, std::string> _props;
};

class FilePropertiesProvider : public PropertiesProvider {
protected:
  static constexpr char Comment_Marker = '#';
  static constexpr char Id_Value_Delimiter = '=';
public:

  FilePropertiesProvider() {}

  void init(const std::string &fname) {
    auto file = std::ifstream{fname};
    auto line = unsigned{0};
    while (true) {
      std::string buf;
      std::getline(file, buf);
      if (!file) { break; }

      ++line;
      if (buf.empty()) { continue; }
      // TODO: trim string
      if (buf[0] == Comment_Marker) { continue; }
      // TODO: add includes

      auto delim = std::find(buf.begin(), buf.end(), Id_Value_Delimiter);
      if (delim == buf.end()) {
        std::cout << "[WARN][FilePropertiesProvider] Unable to parse "
                  << fname << " entry. Line: " << line
                  << ", unable to find delimiter (" << Id_Value_Delimiter
                  << ")." << std::endl;
        continue;
      }

      auto id = std::string{buf.begin(), delim};
      auto value = std::string{delim + 1, buf.end()};
      //TODO: trim id and value

      if (_props.has_property(id)) {
        std::cout << "[WARN][FilePropertiesProvider] Reset property "
                  << id << "." << std::endl;
      }
      _props.set_property(id, value);
    }
  }

  int get_int(const std::string &id, int dflt) const override {
    return _props.get_int(id, dflt);
  }

  double get_dbl(const std::string &id, double dflt) const override {
    return _props.get_dbl(id, dflt);
  }

  str get_str(const std::string &id, const str &dflt) const override {
    return _props.get_str(id, dflt);
  }

  unsigned get_uint(const std::string &id, unsigned dflt) const override {
    return _props.get_uint(id, dflt);
  }

private:
  MapPropertiesProvider _props;
};


#endif
