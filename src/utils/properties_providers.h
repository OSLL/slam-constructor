#ifndef SLAM_CTOR_UTILS_PROPERTIES_PROVIDER_H
#define SLAM_CTOR_UTILS_PROPERTIES_PROVIDER_H

#include <string>
#include <unordered_map>
#include <fstream>
#include <algorithm>
#include <sstream>

// TODO: replace with filesystem on standard update
#include <libgen.h>
#include <cstring>
#include <memory>
/* #include <experimental/filesystem> */

class PropertiesProvider {
protected:
  using str = std::string;
public:
  // TODO: use templates?
  virtual int      get_int(const std::string &id, int dflt) const = 0;
  virtual double   get_dbl(const std::string &id, double dflt) const = 0;
  virtual str      get_str(const std::string &id, const str &dflt) const = 0;
  virtual unsigned get_uint(const std::string &id, unsigned dflt) const = 0;
  virtual bool     get_bool(const std::string &id, bool dflt) const = 0;
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

  bool  get_bool(const std::string &id, bool dflt) const override {
    if (!has_property(id)) { return dflt; }
    auto value = _props.at(id);
    return !(value == "false" || value == "0");
  }

  MapPropertiesProvider &operator+=(const MapPropertiesProvider &that) {
    _props.insert(that._props.begin(), that._props.end());
    return *this;
  }

private:

  std::unordered_map<std::string, std::string> _props;
};

class FilePropertiesProvider : public PropertiesProvider {
protected:
  static constexpr char Comment_Marker = '#';
  static constexpr char Include_Open_Marker = '<';
  static constexpr char Include_Close_Marker = '>';
  static constexpr char Id_Value_Delimiter = '=';
public:

  FilePropertiesProvider() {}

  void append_file_content(const std::string &path) {
    _props += parse_file(path);
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

  bool get_bool(const std::string &id, bool dflt) const override {
    return _props.get_bool(id, dflt);
  }

private:

  MapPropertiesProvider parse_file(const std::string &path) {
    auto props = MapPropertiesProvider{};
    auto file = std::ifstream{path};
    if (!file) {
      std::cout << "[WARN][FilePropertiesProvider] File " << path
                << "does not exist." << std::endl;
      return props;
    }

    auto line = unsigned{0};
    while (true) {
      std::string buf;
      std::getline(file, buf);
      if (!file) { break; }

      ++line;
      if (buf.empty()) { continue; }
      if (buf.front() == Comment_Marker) { continue; }
      if (buf.front() == Include_Open_Marker) {
        // TODO: circular includes detection
        if (buf.back() == Include_Close_Marker) {
          // NB: portability - unix specific; use <filesystem> when available
          auto path_copy = std::unique_ptr<char[]>{
            new char[path.length() + 1]()
          };
          std::strncpy(path_copy.get(), path.c_str(), path.length());
          constexpr char Unix_Path_Separator = '/';
          auto fname_dir = std::string{dirname(path_copy.get())} +
                           Unix_Path_Separator;
          /*
          auto fname_dir = static_cast<std::string>(
            std::experimental::filesystem::path{fname}.remove_filename());
          */

          auto included_path = fname_dir + std::string{std::next(buf.begin()),
                                                       std::prev(buf.end())};
          append_file_content(included_path);
        } else {
          std::cout << "[Warn][FilePropertiesProvider] Line: " << line
                    << ". Include error: last char is not '>'" << std::endl;
        }
        continue;
      }

      auto delim_i = buf.find(Id_Value_Delimiter);
      if (delim_i == std::string::npos) {
        std::cout << "[WARN][FilePropertiesProvider] Unable to parse "
                  << path << " entry. Line: " << line
                  << ", unable to find delimiter (" << Id_Value_Delimiter
                  << ")." << std::endl;
        continue;
      }

      auto id = buf.substr(0, delim_i);
      auto value = buf.substr(delim_i + 1, buf.size() - delim_i - 1);
      //TODO: trim id and value

      if (props.has_property(id)) {
        std::cout << "[WARN][FilePropertiesProvider] Reset property "
                  << id << "." << std::endl;
      }
      props.set_property(id, value);
    }
    return props;
  }

private:
  MapPropertiesProvider _props;
};


#endif
