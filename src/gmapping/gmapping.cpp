#include <memory>
#include <cassert>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#include "../core/sensor_data.h"
#include "../ros/topic_with_transform.h"
#include "../ros/laser_scan_observer.h"
#include "../ros/pose_correction_tf_publisher.h"
#include "../ros/occupancy_grid_publisher.h"
#include "gmapping_particle_filter.h"


unsigned init_particles_nm() {
  int particles_nm;
  ros::param::param<int>("~rbpf_particles_nm", particles_nm, 30);
  assert(0 < particles_nm && "Particles number must be positive");
  return particles_nm;
}

bool is_async_correction() {
  bool async_correction;
  ros::param::param<bool>("~async_correction", async_correction, false);
  return async_correction;
}

using ObservT = sensor_msgs::LaserScan;
using GmappingMap = GmappingWorld::MapType;

int main(int argc, char** argv) {
  ros::init(argc, argv, "gMapping");

  // TODO: setup CostEstimator and OccEstimator
  auto gcs = std::make_shared<GridCellStrategy>(
    std::make_shared<GmappingBaseCell>(),
    std::shared_ptr<ScanCostEstimator>(nullptr),
    std::shared_ptr<CellOccupancyEstimator>(nullptr)
  );
  auto slam = std::make_shared<GmappingParticleFilter>(
    gcs, init_particles_nm());

  ros::NodeHandle nh;
  TopicWithTransform<sensor_msgs::LaserScan> scan_provider{nh,
      "laser_scan", "odom_combined"};
  // TODO: setup scan skip policy via param
  auto scan_obs = std::make_shared<LaserScanObserver>(slam);
  scan_provider.subscribe(scan_obs);

  auto map_publisher = std::make_shared<OccupancyGridPublisher<GmappingMap>>(
    nh.advertise<nav_msgs::OccupancyGrid>("/map", 5));
  slam->subscribe_map(map_publisher);

  using PosePublT = PoseCorrectionTfPublisher<ObservT>;
  auto pose_publisher = std::make_shared<PosePublT>(is_async_correction());
  scan_provider.subscribe(pose_publisher);
  slam->subscribe_pose(pose_publisher);

  ros::spin();
}
