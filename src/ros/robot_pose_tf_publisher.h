#ifndef __SLAM_CTOR_ROBOT_POSE_TF_PUBLISHER_H_INCLUDED
#define __SLAM_CTOR_ROBOT_POSE_TF_PUBLISHER_H_INCLUDED

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "../core/world.h"

class RobotPoseTfPublisher : public WorldPoseObserver {
public: //methods
  RobotPoseTfPublisher(const std::string &tf_map_frame_id,
                            const std::string &tf_robot_frame_id) :
    _tf_map_frame_id{tf_map_frame_id},
    _tf_robot_frame_id{tf_robot_frame_id} {}

  void on_pose_update(const RobotPose &pose) override {
    auto map2robot = tf::Transform{
      tf::createQuaternionFromRPY(0, 0, pose.theta),
      tf::Vector3(pose.x, pose.y, 0.0)};

    auto transform = tf::StampedTransform{map2robot, ros::Time::now(),
                                          _tf_map_frame_id, _tf_robot_frame_id};
    _tf_brcst.sendTransform(transform);
  }

private: // fields
  std::string _tf_map_frame_id, _tf_robot_frame_id;
  tf::TransformBroadcaster _tf_brcst;
};


#endif
