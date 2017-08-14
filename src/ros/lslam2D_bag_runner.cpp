#include <cstdlib>
#include <iostream>
#include <fstream>
#include <memory>
#include <vector>
#include <string>

#include <sensor_msgs/LaserScan.h>

#include "init_utils.h"
#include "bag_topic_with_transform.h"
#include "laser_scan_observer.h"
#include "robot_pose_observers.h"
#include "../utils/map_dumpers.h"
#include "../utils/properties_providers.h"
#include "../slams/viny/init_viny_slam.h"
#include "../slams/tiny/init_tiny_slam.h"
#include "../slams/gmapping/init_gmapping.h"

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
        case Props_Id: props.append_file_content(*arg); break;
        case Bag_Id: bag_fname = *arg; break;
        default: is_valid = false;
        }
        ++mandatory_id;
      } else if (flag == "-p") {
        props.append_file_content(*arg);
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
           << "      [-v] [-t <traj file>] [-m <map file>] \n"
           << "      [-p <extra properties file>]\n";
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

// TODO: consider moving map type to runtime params
template <typename MapType>
void run_slam(std::shared_ptr<LaserScanGridWorld<MapType>> slam,
              const ProgramArgs &args) {
  if (args.traj_dumper) { slam->subscribe_pose(args.traj_dumper); }
  auto lscan_observer = LaserScanObserver{slam, true};

  ros::Time::init();
  BagTopicWithTransform<sensor_msgs::LaserScan> bag{
    args.bag_fname, laser_scan_2D_ros_topic_name(args.props),
    tf_odom_frame_id(args.props)
  };

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
    GridMapToPgmDumber<MapType>::dump_map(map_file, slam->map());
  }
}

int main(int argc, char** argv) {
  auto args = ProgramArgs{}.init(argv);
  if (!args.is_valid) {
    args.print_usage(std::cout);
    std::exit(-1);
  }

  if (args.slam_type == "viny") {
    run_slam<typename VinySlam::MapType>(init_viny_slam(args.props), args);
  } else if (args.slam_type == "tiny") {
    run_slam<typename TinySlam::MapType>(init_tiny_slam(args.props), args);
  } else if (args.slam_type == "gmapping") {
    run_slam<typename Gmapping::MapType>(init_gmapping(args.props), args);
  } else {
    std::cout << "Unkonw slam type: " << args.slam_type << std::endl;
  }
}
