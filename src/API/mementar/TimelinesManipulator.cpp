#include "mementar/API/mementar/TimelinesManipulator.h"

namespace mementar
{

TimelinesManipulator::TimelinesManipulator(ros::NodeHandle* n) : ManagerClient(n)
{
  n_ = n;
}

TimelinesManipulator::~TimelinesManipulator()
{
  for(auto manipulator : manipulators_)
    if(manipulator.second != nullptr)
      delete manipulator.second;
}

bool TimelinesManipulator::waitInit(int32_t timeout)
{
  return ros::service::waitForService("mementar/manage_multi", timeout);
}

TimelineManipulator* TimelinesManipulator::operator[](const std::string& name)
{
  if(manipulators_.find(name) != manipulators_.end())
    return manipulators_[name];
  else
    return nullptr;
}

TimelineManipulator* TimelinesManipulator::get(const std::string& name)
{
  if(manipulators_.find(name) != manipulators_.end())
    return manipulators_[name];
  else
    return nullptr;
}

bool TimelinesManipulator::add(const std::string& name)
{
  if(manipulators_.find(name) != manipulators_.end())
    return true;
  else
  {
    if(ManagerClient::add(name) == false)
      return false;
    else
    {
      TimelineManipulator* tmp = new TimelineManipulator(n_, name);
      manipulators_[name] = tmp;
      return true;
    }
  }
}

bool TimelinesManipulator::del(const std::string& name)
{
  if(manipulators_.find(name) == manipulators_.end())
    return true;
  else
  {
    if(ManagerClient::del(name) == false)
      return false;
    else
    {
      delete manipulators_[name];
      manipulators_.erase(name);
      return true;
    }
  }
}

} // namepsace mementar
