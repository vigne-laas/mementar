#include "mementar/API/mementar/clients/ActionClient.h"

namespace mementar {

bool ActionClient::exist(const std::string& action_name)
{
  mementar::MementarService srv;
  srv.request.action = "exist";
  srv.request.param = action_name;

  return (callStr(srv) != "");
}

std::vector<std::string> ActionClient::getPending()
{
  mementar::MementarService srv;
  srv.request.action = "getPending";

  return call(srv);
}

bool ActionClient::isPending(const std::string& action_name)
{
  mementar::MementarService srv;
  srv.request.action = "isPending";
  srv.request.param = action_name;

  return (callStr(srv) != "");
}

ros::Time ActionClient::getStartStamp(const std::string& action_name)
{
  mementar::MementarService srv;
  srv.request.action = "getStartStamp";
  srv.request.param = action_name;

  return callStamp(srv);
}

ros::Time ActionClient::getEndStamp(const std::string& action_name)
{
  mementar::MementarService srv;
  srv.request.action = "getEndStamp";
  srv.request.param = action_name;

  return callStamp(srv);
}

ros::Time ActionClient::getDuration(const std::string& action_name)
{
  mementar::MementarService srv;
  srv.request.action = "getDuration";
  srv.request.param = action_name;

  return callStamp(srv);
}

std::string ActionClient::getStartFact(const std::string& action_name)
{
  mementar::MementarService srv;
  srv.request.action = "getStartFact";
  srv.request.param = action_name;

  return callStr(srv);
}

std::string ActionClient::getEndFact(const std::string& action_name)
{
  mementar::MementarService srv;
  srv.request.action = "getEndFact";
  srv.request.param = action_name;

  return callStr(srv);
}

std::vector<std::string> ActionClient::getFactsDuring(const std::string& action_name)
{
  mementar::MementarService srv;
  srv.request.action = "getFactsDuring";
  srv.request.param = action_name;

  return call(srv);
}

} // namespace mementar