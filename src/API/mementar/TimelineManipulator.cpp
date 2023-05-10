#include "mementar/API/mementar/TimelineManipulator.h"

namespace mementar
{

TimelineManipulator::TimelineManipulator(ros::NodeHandle* n, const std::string& name) : fact_feeder(n, name),
                                                                                        action_feeder(n, name)
{
  n_ = n;
  name_ = name;
}

bool TimelineManipulator::waitInit(int32_t timeout)
{
  std::string servive_name = (name_ == "") ? "mementar/manage_instance" : "mementar/manage_instance/" + name_;
  return ros::service::waitForService(servive_name, timeout);
}

} // namespace mementar
