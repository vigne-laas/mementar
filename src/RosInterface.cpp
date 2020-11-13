#include "mementar/RosInterface.h"

#include <experimental/filesystem>
#include <thread>

#include <ros/callback_queue.h>

#include "mementar/core/utility/error_code.h"
#include "mementar/graphical/Display.h"

namespace mementar
{

RosInterface::RosInterface(ros::NodeHandle* n, const std::string& directory, const std::string& configuration_file, size_t order, std::string name) :
                                                                                                                onto_(name),
                                                                                                                feeder_(&onto_),
                                                                                                                occasions_(n, name),
                                                                                                                run_(true)

{
  std::cout << "RosInterface" << std::endl;
  n_ = n;
  order_ = order;
  onto_.close();

  if(directory != "none")
  {
    directory_ = directory;
    if(name == "")
      directory_ += "/mementar";
    else
      directory_ += "/" + name;

    std::experimental::filesystem::create_directories(directory_);
  }

  if(configuration_file != "none")
  {
    if(configuration_.read(configuration_file))
    {
      if(configuration_.exist("whitelist"))
      {
        if(feeder_.setWhitelist(configuration_["whitelist"].value()) == false)
          Display::error("A whitelist can not be setted while a blacklist is used");
        else
          Display::info("A whitelist has been setted");
      }
      if(configuration_.exist("blacklist"))
      {
        if(feeder_.setBlacklist(configuration_["blacklist"].value()) == false)
          Display::error("A blacklist can not be setted while a whitelist is used");
        else
          Display::info("A blacklist has been setted");
      }
    }
    else
      Display::error("Fail to load configuartion file : " + configuration_file);
  }

  timeline_ = new Timeline();
  feeder_.link(timeline_);

  name_ = name;
}

RosInterface::~RosInterface()
{
  delete timeline_;
}

void RosInterface::run()
{
  std::string service_name;

  ros::Subscriber knowledge_subscriber = n_->subscribe(getTopicName("insert"), 1000, &RosInterface::knowledgeCallback, this);
  ros::Subscriber stamped_knowledge_subscriber = n_->subscribe(getTopicName("insert_stamped"), 1000, &RosInterface::stampedKnowledgeCallback, this);
  ros::Subscriber explanation_knowledge_subscriber = n_->subscribe(getTopicName("insert_explanations"), 1000, &RosInterface::explanationKnowledgeCallback, this);

  // Start up ROS service with callbacks
  service_name = (name_ == "") ? "actions" : "actions/" + name_;
  ros::ServiceServer service = n_->advertiseService(service_name, &RosInterface::actionsHandle, this);

  feeder_.setCallback([this](const Triplet& triplet){ this->occasions_.add(triplet); });
  std::thread occasions_thread(&OccasionsManager::run, &occasions_);
  std::thread feed_thread(&RosInterface::feedThread, this);

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
  delete timeline_;
  std::experimental::filesystem::remove_all(directory_);
  std::experimental::filesystem::create_directories(directory_);
  timeline_ = new Timeline();
  feeder_.link(timeline_);
  mut_.unlock();
}

void RosInterface::lock()
{
  feeder_mutex_.lock();
}

void RosInterface::release()
{
  feeder_mutex_.unlock();
}


/***************
*
* Callbacks
*
****************/

void RosInterface::knowledgeCallback(const std_msgs::String::ConstPtr& msg)
{
  feeder_.store(msg->data, time(0));
}

void RosInterface::stampedKnowledgeCallback(const StampedString::ConstPtr& msg)
{
  feeder_.store(msg->data, msg->stamp.sec);
}

void RosInterface::explanationKnowledgeCallback(const MementarExplanation::ConstPtr& msg)
{
  feeder_.store(msg->fact, msg->cause);
}

bool RosInterface::actionsHandle(mementar::MementarService::Request &req,
                                 mementar::MementarService::Response &res)
{
  res.code = 0;

  removeUselessSpace(req.action);
  removeUselessSpace(req.param);

  /*if(req.action == "newSession")
  {
    mut_.lock_shared();
    tree_->newSession();
    mut_.unlock_shared();
  }
  else */if(req.action == "reset")
    reset();
  else
    res.code = UNKNOW_ACTION;

  return true;
}

/***************
*
* Threads
*
***************/

void RosInterface::feedThread()
{
  ros::Publisher feeder_publisher = n_->advertise<std_msgs::String>(getTopicName("feeder_notifications"), 1000);

  ros::Rate wait(100);
  while((ros::ok()) && (timeline_->isInit() == false) && (run_ == true))
  {
    wait.sleep();
  }

  std_msgs::String msg;
  while(ros::ok() && (run_ == true))
  {
    feeder_mutex_.lock();
    bool run = feeder_.run();
    if(run == true)
    {
      std::vector<std::string> notifications = feeder_.getNotifications();
      for(auto notif : notifications)
      {
        Display::error(notif);
        if(name_ != "")
          notif = "[" + name_ + "]" + notif;
        msg.data = notif;
        feeder_publisher.publish(msg);
      }
    }
    feeder_mutex_.unlock();

    if(ros::ok() && (run_ == true))
      wait.sleep();
  }
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
