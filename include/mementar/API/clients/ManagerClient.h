#ifndef MEMENTAR_MANAGERCLIENT_H
#define MEMENTAR_MANAGERCLIENT_H

#include "mementar/API/clients/ClientBase.h"

namespace mementar
{

class ManagerClient : public ClientBase
{
public:
  ManagerClient(ros::NodeHandle* n) : ClientBase(n, "manage")
  {
  }

  std::vector<std::string> list();
  bool add(const std::string& name);
  bool del(const std::string& name);

private:

};

} // namespace mementar

#endif // MEMENTAR_MANAGERCLIENT_H
