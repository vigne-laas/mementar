#include "mementar/API/clients/ManagerClient.h"

namespace mementar
{

std::vector<std::string> ManagerClient::list()
{
  mementar::MementarService srv;
  srv.request.action = "list";

  return call(srv);
}

bool ManagerClient::add(const std::string& name)
{
  mementar::MementarService srv;
  srv.request.action = "add";
  srv.request.param = name;

  return callNR(srv);
}

bool ManagerClient::del(const std::string& name)
{
  mementar::MementarService srv;
  srv.request.action = "delete";
  srv.request.param = name;

  return callNR(srv);
}

} // namespace mementar
