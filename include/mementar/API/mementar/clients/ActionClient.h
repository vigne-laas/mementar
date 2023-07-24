#ifndef MEMENTAR_ACTIONCLIENT_H
#define MEMENTAR_ACTIONCLIENT_H

#include "mementar/API/mementar/clients/ClientBase.h"

namespace mementar
{

class ActionClient : public ClientBase
{
public:
  ActionClient(ros::NodeHandle* n, const std::string& name) : ClientBase(n, (name == "") ? "action" : "action/" + name) {}

  bool exist(const std::string& action_name);
  std::vector<std::string> getPending();
  bool isPending(const std::string& action_name);
  ros::Time getStartStamp(const std::string& action_name);
  ros::Time getEndStamp(const std::string& action_name);
  ros::Time getDuration(const std::string& action_name);
  std::string getStartFact(const std::string& action_name);
  std::string getEndFact(const std::string& action_name);
  std::vector<std::string> getFactsDuring(const std::string& action_name);
private:
};

} // namespace mementar

#endif // MEMENTAR_ACTIONCLIENT_H