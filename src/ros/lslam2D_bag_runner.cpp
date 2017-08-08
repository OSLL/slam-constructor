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

int main(int argc, char** argv) {
  if (argc != 6) {
    std::cerr << "Args: <slam type> <properties file> <bag file>"
              << "<traj_file> <map_file>" << std::endl;
    std::exit(-1);
  }

  constexpr auto /*In_Slam_Type = 1,*/ In_Props_Ind = 2, In_Bag_Ind = 3;
  constexpr auto Out_Traj = 4, Out_Map = 5;

  auto props = FilePropertiesProvider{};
  props.init(argv[In_Props_Ind]);
  auto slam = init_viny_slam(props); // FIXME: use slam type for dispatching
  auto traj_dumper = std::make_shared<RobotPoseTumTrajectoryDumper>(
    argv[Out_Traj]);
  slam->subscribe_pose(traj_dumper);
  auto lscan_observer = LaserScanObserver{slam, true};

  ros::Time::init();
  BagTopicWithTransform<sensor_msgs::LaserScan> bag{
    argv[In_Bag_Ind], "/base_scan", "odom_combined"};

  while (bag.extract_next_msg()) {
    lscan_observer.handle_transformed_msg(bag.msg(), bag.transform());
  }
  auto map_file = std::ofstream{argv[Out_Map], std::ios::binary};
  GridMapToPgmDumber<VinySlam::MapType>::dump_map(map_file, slam->map());
}
