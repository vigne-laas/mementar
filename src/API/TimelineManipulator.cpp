#include "mementar/API/TimelineManipulator.h"

namespace mementar
{

TimelineManipulator::TimelineManipulator(ros::NodeHandle* n, const std::string& name) : OccasionsPublisher(n, name)
{
  n_ = n;
  name_ = name;
}

bool TimelineManipulator::waitInit(int32_t timeout)
{
  std::string servive_name = (name_ == "") ? "mementar/actions" : "mementar/actions/" + name_;
  return ros::service::waitForService(servive_name, timeout);
}

} // namespace mementar
