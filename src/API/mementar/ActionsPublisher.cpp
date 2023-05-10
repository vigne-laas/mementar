#include "mementar/API/mementar/ActionsPublisher.h"

#include "mementar/MementarAction.h"

namespace mementar
{

ActionsPublisher::ActionsPublisher(ros::NodeHandle* n, const std::string& name) :
            pub_(n->advertise<MementarAction>((name == "") ? "mementar/insert_action" : "mementar/insert_action/" + name, 1000))
{
  n_ = n;
}

void ActionsPublisher::insert(const std::string& name, time_t start_stamp, time_t end_stamp)
{
  publish(name, start_stamp, end_stamp);
}

void ActionsPublisher::insert(const std::string& name, ros::Time start_stamp, ros::Time end_stamp)
{
  publish(name, start_stamp, end_stamp);
}


void ActionsPublisher::insertEnd(const std::string& name, time_t end_stamp)
{
  publish(name, 0, end_stamp);
}

void ActionsPublisher::publish(const std::string& name, time_t start_stamp, time_t end_stamp)
{
  MementarAction msg;
  msg.name = name;
  msg.start_stamp.sec = start_stamp;
  msg.end_stamp.sec = end_stamp;
  pub_.publish(msg);
}
void ActionsPublisher::publish(const std::string& name, ros::Time start_stamp, ros::Time end_stamp)
{
  MementarAction msg;
  msg.name = name;
  msg.start_stamp.sec = start_stamp.sec;
  msg.start_stamp.nsec = start_stamp.nsec;
  msg.end_stamp.sec = end_stamp.sec;
  msg.end_stamp.nsec = end_stamp.nsec;
  pub_.publish(msg);
}

} // namespace mementar
