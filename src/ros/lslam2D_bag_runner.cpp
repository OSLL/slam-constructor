#include <cstdlib>
#include <iostream>
#include <fstream>
#include <memory>
#include <vector>
#include <string>

#include <sensor_msgs/LaserScan.h>

#include "bag_topic_with_transform.h"
#include "laser_scan_observer.h"
#include "robot_pose_observers.h"
#include "../utils/map_dumpers.h"
#include "../utils/properties_providers.h"
#include "../slams/viny/init_viny_slam.h"

struct ProgramArgs {
  static constexpr auto Slam_Type_Id = 0, Props_Id = 1, Bag_Id = 2,
                        Mandatory_Id_Nm = 3;
  ProgramArgs() : is_valid{true}, is_verbose{false} {}

  // TODO: refactor
  ProgramArgs& init(char **argv) {
    char **arg = argv;
    int mandatory_id = 0;
    std::string flag;
    while (1) {
      ++arg; // skip the first prog name
      if (*arg == 0) { break; }

      if ((*arg)[0] == '-') {
        flag = *arg;
        // handle flags w/o value
        if (flag == "-v") {
          is_verbose = true;
          flag = "";
        }
        continue;
      }

      if (flag == "") {
        switch (mandatory_id) {
        case Slam_Type_Id: slam_type = *arg; break;
        case Props_Id: props.init(*arg); break;
        case Bag_Id: bag_fname = *arg; break;
        default: is_valid = false;
        }
        ++mandatory_id;
      } else if (flag == "-t") {
        traj_dumper = std::make_shared<RobotPoseTumTrajectoryDumper>(*arg);
      } else if (flag == "-m") {
        map_fname = *arg;
      } else {
        std::cout << "Skip parameter for unknown flag \""
                  << flag << "\"" << std::endl;
      }
      flag = "";
    }
    is_valid &= mandatory_id == Mandatory_Id_Nm;
    return *this;
  }

  void print_usage(std::ostream &stream) {
    stream << "Args: <slam type> <properties file> <bag file>\n"
           << "      [-v] [-t <traj file>] [-m <map file>]\n";
  }

  bool is_valid;
// mandatory args
  std::string slam_type;
  FilePropertiesProvider props;
  std::string bag_fname;
// optional args
  std::shared_ptr<RobotPoseTumTrajectoryDumper> traj_dumper;
  std::string map_fname;
  bool is_verbose;
};

int main(int argc, char** argv) {
  auto args = ProgramArgs{}.init(argv);
  if (!args.is_valid) {
    args.print_usage(std::cout);
    std::exit(-1);
  }

  // FIXME: use slam type for dispatching
  auto slam = init_viny_slam(args.props);

  if (args.traj_dumper) { slam->subscribe_pose(args.traj_dumper); }
  auto lscan_observer = LaserScanObserver{slam, true};

  ros::Time::init();
  BagTopicWithTransform<sensor_msgs::LaserScan> bag{
    args.bag_fname, "/base_scan", "odom_combined"};

  auto scan_id = unsigned{0};
  while (bag.extract_next_msg()) {
    lscan_observer.handle_transformed_msg(bag.msg(), bag.transform());
    ++scan_id;
    if (args.is_verbose) {
      std::cout << "Handled scan #" << scan_id << std::endl;
    }
  }

  if (!args.map_fname.empty()) {
    auto map_file = std::ofstream{args.map_fname, std::ios::binary};
    GridMapToPgmDumber<VinySlam::MapType>::dump_map(map_file, slam->map());
  }
}
