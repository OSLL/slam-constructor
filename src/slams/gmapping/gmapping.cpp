#include <memory>
#include <cassert>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#include "../../ros/topic_with_transform.h"
#include "../../ros/laser_scan_observer.h"
#include "../../ros/init_utils.h"

#include "../../ros/launch_properties_provider.h"
#include "../../utils/init_occupancy_mapping.h"
#include "gmapping_scan_probability_estimator.h"
#include "gmapping_particle_filter.h"

auto init_particles_nm(std::shared_ptr<PropertiesProvider> props) {
  return props->get_uint("slam/particles/number", 30);
}

auto init_gmapping_params(std::shared_ptr<PropertiesProvider> props) {
  auto mean_sample_xy = props->get_dbl("slam/particles/sample/xy/mean", 0.0);
  auto sigma_sample_xy = props->get_dbl("slam/particles/sample/xy/sigma", 0.1);
  auto mean_sample_th = props->get_dbl("slam/particles/sample/theta/mean", 0.0);
  auto sigma_sample_th = props->get_dbl("slam/particles/sample/theta/sigma",
                                        0.03);

  auto min_sm_lim_xy = props->get_dbl("slam/particles/sm_delta_lim/xy/min",
                                      0.6);
  auto max_sm_lim_xy = props->get_dbl("slam/particles/sm_delta_lim/xy/max",
                                      0.8);
  auto min_sm_lim_th = props->get_dbl("slam/particles/sm_delta_lim/theta/min",
                                      0.3);
  auto max_sm_lim_th = props->get_dbl("slam/particles/sm_delta_lim/theta/max",
                                      0.4);
  return GMappingParams{mean_sample_xy, sigma_sample_xy,
                        mean_sample_th, sigma_sample_th,
                        min_sm_lim_xy, max_sm_lim_xy,
                        min_sm_lim_th, max_sm_lim_th};
}

using ObservT = sensor_msgs::LaserScan;
using GmappingMap = GmappingWorld::MapType;

int main(int argc, char** argv) {
  ros::init(argc, argv, "GMapping");

  auto props = std::make_shared<LaunchPropertiesProvider>();
  auto gcs = std::make_shared<GridCellStrategy>(
    std::make_shared<GmappingBaseCell>(),
    std::make_shared<GmappingScanProbabilityEstimator>(),
    init_occ_estimator(props)
  );
  auto slam = std::make_shared<GmappingParticleFilter>(
    gcs, init_grid_map_params(props),
    init_gmapping_params(props), init_particles_nm(props));

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
