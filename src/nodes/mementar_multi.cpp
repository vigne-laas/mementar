#include <thread>
#include <regex>

#include <ros/ros.h>

#include "mementar/RosInterface.h"
#include "mementar/core/utility/error_code.h"

void removeUselessSpace(std::string& text)
{
  while((text[0] == ' ') && (text.size() != 0))
    text.erase(0,1);

  while((text[text.size() - 1] == ' ') && (text.size() != 0))
    text.erase(text.size() - 1,1);
}

ros::NodeHandle* n_;
std::map<std::string, mementar::RosInterface*> interfaces_;
std::map<std::string, std::thread> interfaces_threads_;

std::string directory = "";

bool deleteInterface(std::string name)
{
  interfaces_[name]->stop();
  try
  {
    interfaces_threads_[name].join();
  }
  catch(...)
  {
    return false;
  }

  interfaces_threads_.erase(name);
  delete interfaces_[name];
  interfaces_.erase(name);

  std::cout << name << " STOPED" << std::endl;
  return true;
}

bool managerHandle(mementar::MementarService::Request &req,
                   mementar::MementarService::Response &res)
{
  res.code = 0;

  removeUselessSpace(req.action);
  removeUselessSpace(req.param);

  if(req.action == "add")
  {
    auto it = interfaces_.find(req.param);
    if(it != interfaces_.end())
      res.code = NO_EFFECT;
    else
    {
      mementar::RosInterface* tmp = new mementar::RosInterface(n_, directory, 10, req.param);
      interfaces_[req.param] = tmp;
      std::thread th(&mementar::RosInterface::run, tmp);
      interfaces_threads_[req.param] = std::move(th);

      std::cout << req.param << " STARTED" << std::endl;
    }
  }
  else if(req.action == "delete")
  {
    auto it = interfaces_.find(req.param);
    if(it == interfaces_.end())
      res.code = NO_EFFECT;
    else
    {
      if(deleteInterface(req.param) == false)
        res.code = REQUEST_ERROR;
    }
  }
  else if(req.action == "list")
  {
    for(auto it : interfaces_)
      res.values.push_back(it.first);
  }
  else
    res.code = UNKNOW_ACTION;

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mementar_multi");

  ros::NodeHandle n("mementar");
  n_ = &n;

  directory = std::string(argv[1]);
  std::cout << "directory " << directory << std::endl;

  ros::ServiceServer service = n_->advertiseService("manage", managerHandle);

  ros::spin();

  std::vector<std::string> interfaces_names;
  for(auto intreface : interfaces_)
    interfaces_names.push_back(intreface.first);

  for(size_t i = 0; i < interfaces_names.size(); i++)
    deleteInterface(interfaces_names[i]);

  ROS_DEBUG("KILL mementar");

  return 0;
}
