#include "mementar/RosInterface.h"

#include <experimental/filesystem>
#include <thread>

#include <ros/callback_queue.h>

#include "mementar/core/utility/error_code.h"

namespace mementar
{

RosInterface::RosInterface(ros::NodeHandle* n, const std::string& directory, size_t order, std::string name) : occasions_(n, name),
                                                                                                               run_(true)
{
  n_ = n;
  order_ = order;
  directory_ = directory;
  if(name == "")
    directory_ += "/mementar";
  else
    directory_ += "/" + name;

  std::experimental::filesystem::create_directories(directory_);
  tree_ = new ArchivedLeafNode(directory_, order_);

  name_ = name;
}

RosInterface::~RosInterface()
{
  delete tree_;
}

void RosInterface::run()
{
  std::string service_name;

  service_name = (name_ == "") ? "insert" : "insert/" + name_;
  ros::Subscriber knowledge_subscriber = n_->subscribe(service_name, 1000, &RosInterface::knowledgeCallback, this);

  service_name = (name_ == "") ? "insert_stamped" : "insert_stamped/" + name_;
  ros::Subscriber stamped_knowledge_subscriber = n_->subscribe(service_name, 1000, &RosInterface::stampedKnowledgeCallback, this);

  // Start up ROS service with callbacks
  service_name = (name_ == "") ? "actions" : "actions/" + name_;
  ros::ServiceServer service = n_->advertiseService(service_name, &RosInterface::actionsHandle, this);

  std::thread occasions_thread(&OccasionsManager::run, &occasions_);

  ROS_DEBUG("%s mementar ready", name_.c_str());

  while (ros::ok() && isRunning())
  {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  }

  occasions_.stop();
  occasions_thread.join();
}

void RosInterface::reset()
{
  mut_.lock();
  delete tree_;
  std::experimental::filesystem::remove_all(directory_);
  std::experimental::filesystem::create_directories(directory_);
  tree_ = new ArchivedLeafNode(directory_, order_);
  mut_.unlock();
}

/***************
*
* Callbacks
*
****************/

void RosInterface::knowledgeCallback(const std_msgs::String::ConstPtr& msg)
{
  LinkedFact* fact = new LinkedFact(msg->data);
  if(fact->valid())
  {
    mut_.lock_shared();
    tree_->insert(time(0), fact);
    mut_.unlock_shared();
    occasions_.add(fact);
  }
}

void RosInterface::stampedKnowledgeCallback(const StampedString::ConstPtr& msg)
{
  LinkedFact* fact = new LinkedFact(msg->data);
  if(fact->valid())
  {
    mut_.lock_shared();
    tree_->insert(msg->stamp.sec, fact);
    mut_.unlock_shared();
    occasions_.add(fact);
  }
}

bool RosInterface::actionsHandle(mementar::MementarService::Request &req,
                                 mementar::MementarService::Response &res)
{
  res.code = 0;

  removeUselessSpace(req.action);
  removeUselessSpace(req.param);

  if(req.action == "remove")
  {
    LinkedFact fact(req.param);
    if(fact.valid())
    {
      mut_.lock_shared();
      tree_->remove(req.stamp.sec, req.param);
      mut_.unlock_shared();
    }
    else
      res.code = REQUEST_ERROR;
  }
  else if(req.action == "newSession")
  {
    mut_.lock_shared();
    tree_->newSession();
    mut_.unlock_shared();
  }
  else if(req.action == "reset")
    reset();
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
