#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <vector>


// default parameters (these can be, probably, customized)

#define DEFAULT_PATH_FRAME_ID "/map"
#define DEFAULT_ROBOT_FRAME_ID "/base_link"
#define DEFAULT_WORLD_FRAME_ID "/map"
#define DEFAULT_TOPIC_NAME "/path"
#define DEFAULT_NODE_NAME "path_publisher"

// private parameters

#define FREQUENCY 5
#define PUB_QUEUE_SIZE 5
#define TF_WAIT_TIME 3


// Returns stamped base_link pose.
// In case of failure reports an error and does retries.
// If ros::shutdown is issued, will return a dummy pose object.
static geometry_msgs::PoseStamped getCurrentPose() {
	tf::TransformListener tf_listener;
    tf::StampedTransform stamped_transform;

	while (ros::ok()) {
		try {
			auto stamp_latest = ros::Time(0);

			tf_listener.waitForTransform(DEFAULT_WORLD_FRAME_ID, DEFAULT_ROBOT_FRAME_ID, stamp_latest, ros::Duration(TF_WAIT_TIME));
			tf_listener.lookupTransform(DEFAULT_WORLD_FRAME_ID, DEFAULT_ROBOT_FRAME_ID, stamp_latest, stamped_transform);

			geometry_msgs::PoseStamped pose_stamped;
			pose_stamped.header.frame_id = stamped_transform.frame_id_;
			pose_stamped.header.stamp = stamped_transform.stamp_;
			pose_stamped.pose.position.x = stamped_transform.getOrigin().x();
			pose_stamped.pose.position.y = stamped_transform.getOrigin().y();

			return pose_stamped;
		}
		catch (tf::TransformException &ex) {
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
	}

	return geometry_msgs::PoseStamped();
}


int main(int argc, char **argv) {
 	ros::init(argc, argv, DEFAULT_NODE_NAME);

    auto node_handle = ros::NodeHandle();
    auto publisher = node_handle.advertise<nav_msgs::Path>(DEFAULT_TOPIC_NAME, PUB_QUEUE_SIZE);
	auto loop_rate = ros::Rate(FREQUENCY);

    nav_msgs::Path path;
	path.header.frame_id = DEFAULT_WORLD_FRAME_ID;

	while(node_handle.ok()) {
		auto pose_stamped = getCurrentPose();
		path.header.stamp = pose_stamped.header.stamp;
		path.poses.push_back(pose_stamped);

		publisher.publish(path);

		loop_rate.sleep();
	}

    return 0;
}
