#ifndef MEMENTAR_FEEDERECHO_H
#define MEMENTAR_FEEDERECHO_H

#include <mutex>

#include <ros/ros.h>
#include "mementar/StampedFact.h"
#include "mementar/core/memGraphs/Branchs/ContextualizedFact.h"

namespace mementar {

class FeederEcho
{
public:
  explicit FeederEcho(const std::string& echo_topic) : feeder_echo_pub_(n_.advertise<mementar::StampedFact>(echo_topic, 1000))
  {}

  ~FeederEcho()
  {
    mut_.lock();
    echo_messages.clear();
  }

  void add(ContextualizedFact* fact)
  {
    mut_.lock();
    echo_messages.emplace_back(fact);
    mut_.unlock();
  }

  void publish()
  {
    mut_.lock();
    mementar::StampedFact ros_msg;
    for(auto& message : echo_messages)
    {
      ros_msg.id = message->getId();
      ros_msg.stamp = ros::Time(message->getTime());
      ros_msg.subject = message->subject_;
      ros_msg.predicat = message->predicat_;
      ros_msg.object = message->object_;
      ros_msg.added = message->add_;
      feeder_echo_pub_.publish(ros_msg);
    }
    echo_messages.clear();
    mut_.unlock();
  }

private:
  std::mutex mut_;
  ros::NodeHandle n_;
  ros::Publisher feeder_echo_pub_;
  std::vector<ContextualizedFact*> echo_messages;
};

} // namespace mementar

#endif // MEMENTAR_FEEDERECHO_H
