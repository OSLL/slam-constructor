#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <algorithm>
#include <iterator>

#include "../core/states/robot_pose.h"
#include "../core/states/sensor_data.h"
#include "properties_providers.h"
#include "init_scan_matching.h"
#include "init_occupancy_mapping.h"
#include "map_dumpers.h"

constexpr std::size_t Config_File_I = 1, Pose_File_I = 2, Map_File_I = 3,
                      Scan_File_I = 4, Arg_Nm = 5;

void print_usage() {
  std::cout << "Usage: sm_runner <config.properties> <file.pose2D>"
            << " <file.map> <file.scan2D>" << std::endl;
}

decltype(auto) setup_pose2D(const std::string &fname) {
  auto pose_fstrm = std::ifstream{fname};
  if (!pose_fstrm.good()) {
    std::cout << "Unable to read pose from " << fname << std::endl;
    std::exit(-1);
  }
  double x = 0, y = 0, theta = 0;
  pose_fstrm >> x >> y >> theta;
  auto pose = RobotPose{x, y, theta};
  pose_fstrm.close();
  return pose;
}

decltype(auto) setup_map(const PropertiesProvider &props,
                         const std::string &map_fname) {
  auto map_fstrm = std::ifstream{map_fname};
  if (!map_fstrm.good()) {
    std::cout << "Unable to read map from " << map_fname << std::endl;
    std::exit(-1);
  }

  auto map = init_grid_map(props);
  auto buf = std::vector<char>{};
  std::copy(std::istreambuf_iterator<char>(map_fstrm),
            std::istreambuf_iterator<char>(), std::back_inserter(buf));
  map->load_state(buf);
  return map;
}

decltype(auto) setup_laser_scan2D(const std::string &fname) {
  auto scan_fstrm = std::ifstream{fname};
  if (!scan_fstrm.good()) {
    std::cout << "Unable to read scan from " << fname << std::endl;
    std::exit(-1);
  }
  auto scan = LaserScan2D{};
  scan_fstrm >> scan;
  scan_fstrm.close();
  return scan;
}

int main(int argc, char **argv) {
  if (argc != Arg_Nm) {
    print_usage();
    return -1;
  }

  auto props = FilePropertiesProvider{};
  props.append_file_content(argv[Config_File_I]);
  //assert(is_m3rsm(scan_matcher_type(props)));

  // 1) Setup
  auto pose = setup_pose2D(argv[Pose_File_I]);
  auto raw_scan = setup_laser_scan2D(argv[Scan_File_I]);
  auto map = setup_map(props, argv[Map_File_I]);
  // TODO: validate map

  // 2)
  auto sm = init_scan_matcher(props);
  auto scan = sm->filter_scan(raw_scan, pose, *map);
  auto pose_delta = RobotPoseDelta{0, 0, 0};

  auto tr_scan = TransformedLaserScan{{}, scan, 1 /* qualtiy */};
  auto pose_prob = sm->process_scan(tr_scan, pose, *map, pose_delta);

  std::cout << "Pose delta: " << pose_delta
            << " with probability " << pose_prob << std::endl;
  return 0;
}
