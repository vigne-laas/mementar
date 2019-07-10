#ifndef MEMENTAR_TIMELINEMANIPULATOR_H
#define MEMENTAR_TIMELINEMANIPULATOR_H

#include <string>

#include <ros/ros.h>

#include "mementar/API/EventsPublisher.h"
#include "mementar/API/clients/ClientBase.h"

namespace mementar
{

class TimelineManipulator : public EventsPublisher
{
public:
  TimelineManipulator(ros::NodeHandle* n, const std::string& name = "");

  bool waitInit(int32_t timeout = -1);

  /*size_t nb() {return actions.nb();}
  void resetNb() {actions.resetNb();}*/

  void verbose(bool verbose) { ClientBase::verbose(verbose); }

private:
  ros::NodeHandle* n_;
  std::string name_;
};

} // namespace mementar

#endif // MEMENTAR_TIMELINEMANIPULATOR_H
