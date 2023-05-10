#ifndef MEMENTAR_TIMELINEMANIPULATOR_H
#define MEMENTAR_TIMELINEMANIPULATOR_H

#include <string>

#include <ros/ros.h>

#include "mementar/API/mementar/ActionsPublisher.h"
#include "mementar/API/mementar/OccasionsPublisher.h"
#include "mementar/API/mementar/clients/ClientBase.h"
#include "mementar/API/mementar/clients/ActionClient.h"
#include "mementar/API/mementar/clients/FactClient.h"

namespace mementar
{

class TimelineManipulator
{
public:
  TimelineManipulator(ros::NodeHandle* n, const std::string& name = "");

  bool waitInit(int32_t timeout = -1);

  /*size_t nb() {return actions.nb();}
  void resetNb() {actions.resetNb();}*/

  void verbose(bool verbose) { ClientBase::verbose(verbose); }

  OccasionsPublisher fact_feeder;
  ActionsPublisher action_feeder;
  ActionClient actions;
  FactClient facts;

private:
  ros::NodeHandle* n_;
  std::string name_;
};

} // namespace mementar

#endif // MEMENTAR_TIMELINEMANIPULATOR_H
