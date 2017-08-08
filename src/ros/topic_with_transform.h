#ifndef SLAM_CTOR_ROS_TOPIC_WITH_TRANSFORM_H
#define SLAM_CTOR_ROS_TOPIC_WITH_TRANSFORM_H

#include <string>
#include <vector>
#include <memory>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <boost/shared_ptr.hpp>

// TODO: make this class inner
template <typename MType>
class TopicObserver { // iface
public: // methods
  virtual void handle_transformed_msg(const boost::shared_ptr<MType>,
                                      const tf::StampedTransform&) = 0;
};

// TODO: add scan drop
template <typename MsgType>
class TopicWithTransform {
  /* NB: wasn't able to implement with TF2 (ROS jade),
         probably because of deadlock
           (https://github.com/ros/geometry2/pull/144)
         Try TF2 with proposed patch.
   */
public: // methods
  // TODO: copy, move ctrs, dtor
  TopicWithTransform(ros::NodeHandle nh,
                     const std::string& topic_name,
                     const std::string& target_frame,
                     const double buffer_duration = 5.0,
                     const uint32_t tf_filter_queue_size = 1000,
                     const uint32_t subscribers_queue_size = 1000):
    _target_frame{target_frame},
    _subscr{nh, topic_name, subscribers_queue_size},
    _tf_lsnr{ros::Duration(buffer_duration)},
    _msg_flt{new tf::MessageFilter<MsgType>{
      _subscr, _tf_lsnr, _target_frame, tf_filter_queue_size}} {
      _msg_flt->registerCallback(&TopicWithTransform::transformed_msg_cb,
                                  this);
  }

  void subscribe(std::shared_ptr<TopicObserver<MsgType>> obs) {
    _observers.push_back(obs);
  }
private: // methods
  void transformed_msg_cb(const boost::shared_ptr<MsgType> msg) {
    tf::StampedTransform transform;
    std::string msg_frame_id =
      ros::message_traits::FrameId<MsgType>::value(*msg);
    ros::Time msg_time =
      ros::message_traits::TimeStamp<MsgType>::value(*msg);

    try{
      // NB: using last available msg (time = 0) causes issues
      //     on running with higher rate.
      _tf_lsnr.lookupTransform(_target_frame, msg_frame_id,
                               msg_time, transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("Transform retrieval failed. %s",ex.what());
      return;
    }

    for (auto obs : _observers) {
      if (auto obs_ptr = obs.lock()) {
        obs_ptr->handle_transformed_msg(msg, transform);
      }
    }
  }
private: // fields
  std::string _target_frame;
  message_filters::Subscriber<MsgType> _subscr;
  tf::TransformListener _tf_lsnr;
  std::unique_ptr<tf::MessageFilter<MsgType>> _msg_flt;
  std::vector<std::weak_ptr<TopicObserver<MsgType>>> _observers;
};

#endif // macro guard
