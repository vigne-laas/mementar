#include "mementar/API/mementar/clients/FactClient.h"

namespace mementar {

bool FactClient::exist(const std::string& fact_id)
{
  mementar::MementarService srv;
  srv.request.action = "exist";
  srv.request.param = fact_id;

  return (callStr(srv) != "");
}

bool FactClient::isActionPart(const std::string& fact_id)
{
  mementar::MementarService srv;
  srv.request.action = "isActionPart";
  srv.request.param = fact_id;

  return (callStr(srv) != "");
}

std::string FactClient::getActionPart(const std::string& fact_id)
{
  mementar::MementarService srv;
  srv.request.action = "getActionPart";
  srv.request.param = fact_id;

  return callStr(srv);
}

std::string FactClient::getData(const std::string& fact_id)
{
  mementar::MementarService srv;
  srv.request.action = "getData";
  srv.request.param = fact_id;

  return callStr(srv);
}

ros::Time FactClient::getStamp(const std::string& fact_id)
{
  mementar::MementarService srv;
  srv.request.action = "getStamp";
  srv.request.param = fact_id;

  return callStamp(srv);
}

} // namespace mementar