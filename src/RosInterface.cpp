#include "mementar/RosInterface.h"

#include <experimental/filesystem>

#include <ros/callback_queue.h>

#include "mementar/core/utility/error_code.h"

namespace mementar
{

RosInterface::RosInterface(ros::NodeHandle* n, const std::string& directory, size_t order, std::string name) : run_(true)
{
  n_ = n;

  std::string dir = directory;
  if(name == "")
    dir += "/mementar";
  else
    dir += "/" + name;

  std::experimental::filesystem::create_directory(dir);
  tree_ = new ArchivedLeafNode(dir, order);

  name_ = name;
}

RosInterface::~RosInterface()
{
  delete tree_;
}

void RosInterface::run()
{
  std::string service_name;

  service_name = (name_ == "") ? "mementar/insert" : "mementar/insert/" + name_;
  ros::Subscriber knowledge_subscriber = n_->subscribe(service_name, 1000, &RosInterface::knowledgeCallback, this);

  service_name = (name_ == "") ? "mementar/insert_stamped" : "mementar/insert_stamped/" + name_;
  ros::Subscriber stamped_knowledge_subscriber = n_->subscribe(service_name, 1000, &RosInterface::stampedKnowledgeCallback, this);

  // Start up ROS service with callbacks
  service_name = (name_ == "") ? "mementar/actions" : "mementar/actions/" + name_;
  ros::ServiceServer service = n_->advertiseService(service_name, &RosInterface::actionsHandle, this);

  ROS_DEBUG("%s mementar ready", name_.c_str());

  while (ros::ok() && isRunning())
  {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  }
}

/***************
*
* Callbacks
*
****************/

void RosInterface::knowledgeCallback(const std_msgs::String::ConstPtr& msg)
{
  Fact fact(msg->data);
  if(fact.valid())
    tree_->insert(time(0), fact);
}

void RosInterface::stampedKnowledgeCallback(const StampedString::ConstPtr& msg)
{
  Fact fact(msg->data);
  if(fact.valid())
    tree_->insert(msg->stamp.sec, fact);
}

bool RosInterface::actionsHandle(mementar::MementarService::Request &req,
                                 mementar::MementarService::Response &res)
{
  res.code = 0;

  removeUselessSpace(req.action);
  removeUselessSpace(req.param);

  if(req.action == "remove")
  {
    Fact fact(req.param);
    if(fact.valid())
      tree_->remove(req.stamp.sec, req.param);
    else
      res.code = REQUEST_ERROR;
  }
  else if(req.action == "newSession")
    tree_->newSession();
  else
    res.code = UNKNOW_ACTION;

  return true;
}

/***************
*
* Utility
*
****************/

void RosInterface::removeUselessSpace(std::string& text)
{
  while((text[0] == ' ') && (text.size() != 0))
    text.erase(0,1);

  while((text[text.size() - 1] == ' ') && (text.size() != 0))
    text.erase(text.size() - 1,1);
}

} // namespace mementar
