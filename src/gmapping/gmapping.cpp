#include <memory>
#include <cassert>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#include "../core/sensor_data.h"
#include "../ros/topic_with_transform.h"
#include "../ros/laser_scan_observer.h"
#include "../ros/init_utils.h"

#include "gmapping_particle_filter.h"

unsigned init_particles_nm() {
  int particles_nm;
  ros::param::param<int>("~rbpf_particles_nm", particles_nm, 30);
  assert(0 < particles_nm && "Particles number must be positive");
  return particles_nm;
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
  auto scan_provider = std::make_unique<TopicWithTransform<ObservT>>(
    nh, "laser_scan", tf_odom_frame_id()
  );
  // TODO: setup scan skip policy via param
  auto scan_obs_pin = std::make_shared<LaserScanObserver>(
    slam, init_skip_exceeding_lsr());
  scan_provider->subscribe(scan_obs_pin);

  auto occup_grid_pub_pin = create_occupancy_grid_publisher<GmappingMap>(
    slam.get(), nh);

  auto pose_pub_pin = create_pose_correction_tf_publisher<ObservT, GmappingMap>(
    slam.get(), scan_provider.get());

  ros::spin();
}
