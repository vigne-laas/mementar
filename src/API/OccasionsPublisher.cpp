#include "mementar/API/OccasionsPublisher.h"

#include "mementar/StampedString.h"

namespace mementar
{

OccasionsPublisher::OccasionsPublisher(ros::NodeHandle* n, const std::string& name) :
            pub_(n->advertise<StampedString>((name == "") ? "mementar/insert_fact_stamped" : "mementar/insert_fact_stamped/" + name, 1000))
{
  n_ = n;
}

void OccasionsPublisher::insert(const Fact& fact, time_t stamp)
{
  publish(fact(), stamp);
}

void OccasionsPublisher::publish(const std::string& str, time_t stamp)
{
  StampedString msg;
  msg.data = str;
  msg.stamp.sec = stamp;
  pub_.publish(msg);
}

} // namespace mementar
