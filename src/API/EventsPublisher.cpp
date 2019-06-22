#include "mementar/API/EventsPublisher.h"

#include "mementar/StampedString.h"

namespace mementar
{

EventsPublisher::EventsPublisher(ros::NodeHandle* n, const std::string& name) :
            pub_(n->advertise<StampedString>((name == "") ? "mementar/insert_stamped" : "mementar/insert_stamped/" + name, 1000))
{
  n_ = n;
}

void EventsPublisher::insert(const std::string& subject, const std::string& predicat, const std::string& object, time_t stamp)
{
  publish(subject + "|" + predicat + "|" + object, stamp);
}

void EventsPublisher::insert(const std::string& data, time_t stamp)
{
  publish(data, stamp);
}

void EventsPublisher::publish(const std::string& str, time_t stamp)
{
  StampedString msg;
  msg.data = str;
  if(stamp == 0)
    msg.stamp.sec = time(0);
  else
    msg.stamp.sec = stamp;
  pub_.publish(msg);
}

} // namespace mementar
