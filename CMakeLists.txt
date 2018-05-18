cmake_minimum_required(VERSION 2.8.12)
project(slam_constructor)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -pthread -fdiagnostics-color")

# Replace with CMAKE_CXX_STANDARD when CMake 3.1 become available in ROS buildfarm  
add_compile_options(-std=c++14)
#set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf
  message_filters
  rosbag_storage
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(wg_pr2_bag_adapter src/ros/wg_pr2_bag_adapter.cpp)
add_executable(lslam2D_bag_runner src/ros/lslam2D_bag_runner.cpp)
add_executable(gmapping src/slams/gmapping/gmapping.cpp)
add_executable(tiny_slam src/slams/tiny/tiny_slam.cpp)
add_executable(viny_slam src/slams/viny/viny_slam.cpp)
#add_executable(viny_slam_x src/slams/vinyx/vinyx_slam.cpp)
add_executable(path_publisher src/ros/path_publisher.cpp)
add_executable(p2D_ss_evaluator src/utils/pose2D_search_space_evaluator.cpp)

target_link_libraries(wg_pr2_bag_adapter ${catkin_LIBRARIES})
target_link_libraries(lslam2D_bag_runner ${catkin_LIBRARIES})
target_link_libraries(gmapping ${catkin_LIBRARIES})
target_link_libraries(tiny_slam ${catkin_LIBRARIES})
target_link_libraries(viny_slam ${catkin_LIBRARIES})
#target_link_libraries(viny_slam_x ${catkin_LIBRARIES})
target_link_libraries(path_publisher ${catkin_LIBRARIES})

install(
  TARGETS wg_pr2_bag_adapter lslam2D_bag_runner gmapping tiny_slam viny_slam path_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(GTest)
  include_directories(${GTEST_INCLUDE_DIRS})

  # Maps
  catkin_add_gtest(area_occupancy_estimator-test
                   test/core/maps/area_occupancy_estimator_test.cpp)
  catkin_add_gtest(gm_scan_adders-test
                   test/core/maps/grid_map_scan_adders_test.cpp)
  catkin_add_gtest(unbounded_plain_grid_map-test
                   test/core/maps/unbounded_plain_grid_map_test.cpp)
  catkin_add_gtest(unbounded_lazy_tiled_grid_maps-test
                   test/core/maps/unbounded_lazy_tiled_grid_map_test.cpp)
  catkin_add_gtest(regular_squares_grid-test
                   test/core/maps/regular_squares_grid_test.cpp)
  catkin_add_gtest(grid_rasterization-test
                   test/core/maps/grid_rasterization_test.cpp)
  catkin_add_gtest(rescalable_caching_grid_map-test
                   test/core/maps/rescalable_caching_grid_map_test.cpp)

  # Core common
  catkin_add_gtest(trig_utils-test
                   test/core/trigonometry_utils_test.cpp)
  catkin_add_gtest(lwrectangle-test
                   test/core/light_weight_rectangle_test.cpp)
  catkin_add_gtest(geom_dprimitives-test
                   test/core/geometry_discrete_primitives_test.cpp)

  # Core states
  catkin_add_gtest(sensor_data-test
                   test/core/states/sensor_data_test.cpp)

  # Features
  catkin_add_gtest(angle_histogram-test
                   test/core/features/angle_histogram_test.cpp)

  # Scan Matching
  catkin_add_gtest(occupancy_observation_probability-test
                   test/core/scan_matchers/occupancy_observation_probability_test.cpp)
  catkin_add_gtest(hill_climbing_sm-smoke_test
                   test/core/scan_matchers/hill_climbing_sm_smoke_test.cpp)
  catkin_add_gtest(brute_force_sm-smoke_test
                   test/core/scan_matchers/brute_force_sm_smoke_test.cpp)
  catkin_add_gtest(bfmrsm-smoke_test
                   test/core/scan_matchers/bf_multi_res_sm_smoke_test.cpp)

  # Utils common
  catkin_add_gtest(grid_map_patcher-test
                   test/utils/data_generation/grid_map_patcher_test.cpp)
  catkin_add_gtest(map_primitives-test
                   test/utils/data_generation/map_primitives_test.cpp)
  catkin_add_gtest(lscan_generator-test
                   test/utils/data_generation/laser_scan_generator_test.cpp)

endif()
