#include <thread>
#include <regex>

#include <ros/ros.h>

#include "mementar/RosInterface.h"
#include "mementar/core/utility/error_code.h"
#include "mementar/core/Parametrization/Parameters.h"

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

mementar::Parameters params;

bool deleteInterface(std::string name)
{
  interfaces_[name]->stop();
  usleep(1000);
  try
  {
    interfaces_threads_[name].join();
  }
  catch(std::runtime_error& ex)
  {
    mementar::Display::error("Catch error when joining the interface thread : " + std::string(ex.what()));
    mementar::Display::warning("The thread will be detached");
    interfaces_threads_[name].detach();
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
      mementar::RosInterface* tmp = new mementar::RosInterface(n_,
                                                               params.parameters_.at("directory").getFirst(),
                                                               params.parameters_.at("config").getFirst(),
                                                               10,
                                                               req.param);
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

  params.insert(mementar::Parameter("directory", {"-d", "--directory"}, {"none"}));
  params.insert(mementar::Parameter("config", {"-c", "--config"}, {"none"}));

  params.set(argc, argv);
  params.display();

  ros::ServiceServer service = n_->advertiseService("manage", managerHandle);
  (void)service;

  ros::spin();

  std::vector<std::string> interfaces_names;
  std::transform(interfaces_.cbegin(), interfaces_.cend(), std::back_inserter(interfaces_names), [](const auto& interface){ return interface.first; });

  for(size_t i = 0; i < interfaces_names.size(); i++)
    deleteInterface(interfaces_names[i]);

  ROS_DEBUG("KILL mementar");

  return 0;
}
