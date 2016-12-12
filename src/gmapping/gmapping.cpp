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
  ros::param::param<int>("~slam/particles/number", particles_nm, 30);
  assert(0 < particles_nm && "Particles number must be positive");
  return particles_nm;
}

GMappingParams init_gmapping_params() {
  double x0_sample_xy, sigma_sample_xy,
         x0_sample_th, sigma_sample_th,
         x0_init_pd_xy, sigma_init_pd_xy,
         x0_init_pd_th, sigma_init_pd_th;
  ros::param::param<double>("slam/particles/sample/xy/x0",
                                           x0_sample_xy, 0.0);
  ros::param::param<double>("slam/particles/sample/xy/sigma",
                                           sigma_sample_xy, 0.1);
  ros::param::param<double>("slam/particles/sample/theta/x0",
                                           x0_sample_th, 0.0);
  ros::param::param<double>("slam/particles/sample/theta/sigma",
                                           sigma_sample_th, 0.03);
  ros::param::param<double>("slam/particles/init_pose_delta/xy/x0",
                                           x0_init_pd_xy, 0.6);
  ros::param::param<double>("slam/particles/init_pose_delta/xy/sigma",
                                           sigma_init_pd_xy, 0.8);
  ros::param::param<double>("slam/particles/init_pose_delta/theta/x0",
                                           x0_init_pd_th, 0.3);
  ros::param::param<double>("slam/particles/init_pose_delta/theta/sigma",
                                           sigma_init_pd_th, 0.4);
  return {x0_sample_xy, sigma_sample_xy,
          x0_sample_th, sigma_sample_th,
          x0_init_pd_xy, sigma_init_pd_xy,
          x0_init_pd_th, sigma_init_pd_th};
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
    gcs, init_grid_map_params(), init_gmapping_params(), init_particles_nm());

  ros::NodeHandle nh;
  double ros_map_publishing_rate, ros_tf_buffer_size;
  int ros_filter_queue, ros_subscr_queue;
  init_constants_for_ros(ros_tf_buffer_size, ros_map_publishing_rate,
                         ros_filter_queue, ros_subscr_queue);
  auto scan_provider = std::make_unique<TopicWithTransform<ObservT>>(
    nh, "laser_scan", tf_odom_frame_id(), ros_tf_buffer_size,
    ros_filter_queue, ros_subscr_queue
  );
  // TODO: setup scan skip policy via param
  auto scan_obs_pin = std::make_shared<LaserScanObserver>(
    slam, init_skip_exceeding_lsr());
  scan_provider->subscribe(scan_obs_pin);

  auto occup_grid_pub_pin = create_occupancy_grid_publisher<GmappingMap>(
    slam.get(), nh, ros_map_publishing_rate);

  auto pose_pub_pin = create_pose_correction_tf_publisher<ObservT, GmappingMap>(
    slam.get(), scan_provider.get());

  ros::spin();
}
