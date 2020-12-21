#include "mementar/RosInterface.h"

#include <experimental/filesystem>
#include <thread>

#include <ros/callback_queue.h>

#include "mementar/core/utility/error_code.h"
#include "mementar/graphical/Display.h"
#include "mementar/graphical/timeline/TimelineDrawer.h"

#include "mementar/utils/String.h"

namespace mementar
{

RosInterface::RosInterface(ros::NodeHandle* n, const std::string& directory, const std::string& configuration_file, size_t order, std::string name) :
                                                                                                                onto_(name),
                                                                                                                feeder_(&onto_),
                                                                                                                occasions_(n, &onto_, name),
                                                                                                                run_(true)

{
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
  ros::Subscriber knowledge_subscriber = n_->subscribe(getTopicName("insert_fact"), 1000, &RosInterface::knowledgeCallback, this);
  ros::Subscriber stamped_knowledge_subscriber = n_->subscribe(getTopicName("insert_fact_stamped"), 1000, &RosInterface::stampedKnowledgeCallback, this);
  ros::Subscriber explanation_knowledge_subscriber = n_->subscribe(getTopicName("insert_fact_explanations"), 1000, &RosInterface::explanationKnowledgeCallback, this);
  ros::Subscriber action_knowledge_subscriber = n_->subscribe(getTopicName("insert_action"), 1000, &RosInterface::actionKnowledgeCallback, this);
  ros::Subscriber onto_stamped_knowledge_subscriber = n_->subscribe(getOntoTopicName("insert_echo"), 1000, &RosInterface::ontoStampedKnowledgeCallback, this);
  ros::Subscriber onto_explanation_knowledge_subscriber = n_->subscribe(getOntoTopicName("insert_explanations"), 1000, &RosInterface::ontoExplanationKnowledgeCallback, this);

  // Start up ROS service with callbacks
  ros::ServiceServer manage_instance_service = n_->advertiseService(getTopicName("manage_instance"), &RosInterface::managerInstanceHandle, this);
  ros::ServiceServer action_service = n_->advertiseService(getTopicName("action"), &RosInterface::actionHandle, this);
  ros::ServiceServer fact_service = n_->advertiseService(getTopicName("fact"), &RosInterface::factHandle, this);

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
  feeder_.storeFact(msg->data, time(0));
}

void RosInterface::stampedKnowledgeCallback(const StampedString::ConstPtr& msg)
{
  feeder_.storeFact(msg->data, msg->stamp.sec);
}

void RosInterface::explanationKnowledgeCallback(const MementarExplanation::ConstPtr& msg)
{
  feeder_.storeFact(msg->fact, msg->cause);
}

void RosInterface::actionKnowledgeCallback(const MementarAction::ConstPtr& msg)
{
  feeder_.storeAction(msg->name,
                      (msg->start_stamp.sec != 0) ? msg->start_stamp.sec : SoftPoint::default_time,
                      (msg->end_stamp.sec != 0) ? msg->end_stamp.sec : SoftPoint::default_time);
}

void RosInterface::ontoStampedKnowledgeCallback(const StampedString::ConstPtr& msg)
{
  feeder_.storeFact(msg->data, msg->stamp.sec);
}

void RosInterface::ontoExplanationKnowledgeCallback(const MementarExplanation::ConstPtr& msg)
{
  feeder_.storeFact(msg->fact, msg->cause);
}

/***************
*
* Services
*
****************/

bool RosInterface::managerInstanceHandle(mementar::MementarService::Request &req,
                                         mementar::MementarService::Response &res)
{
  res.code = 0;

  removeUselessSpace(req.action);
  removeUselessSpace(req.param);
  param_t params = getParams(req.param);

  /*if(req.action == "newSession")
  {
    mut_.lock_shared();
    tree_->newSession();
    mut_.unlock_shared();
  }
  else */if(req.action == "reset")
    reset();
  if(req.action == "draw")
  {
    TimelineDrawer drawer;
    if(drawer.draw(req.param, timeline_) == false)
      res.code = NO_EFFECT;
  }
  else
    res.code = UNKNOW_ACTION;

  return true;
}

bool RosInterface::actionHandle(mementar::MementarService::Request &req,
                                mementar::MementarService::Response &res)
{
  res.code = 0;

  removeUselessSpace(req.action);
  removeUselessSpace(req.param);
  param_t params = getParams(req.param);

  std::unordered_set<std::string> set_res;

  if(req.action == "exist")
  {
    if(timeline_->actions.exist(params()))
      res.values.push_back(params());
  }
  else if(req.action == "getPending")
    set_res = timeline_->actions.getPending();
  else if(req.action == "isPending")
  {
    if(timeline_->actions.isPending(params()))
      res.values.push_back(params());
  }
  else if(req.action == "getStartStamp")
    res.time_value = ros::Time(timeline_->actions.getStartStamp(params()));
  else if(req.action == "getEndStamp")
    res.time_value = ros::Time(timeline_->actions.getEndStamp(params()));
  else if(req.action == "getDuration")
    res.time_value = ros::Time(timeline_->actions.getDuration(params()));
  else if(req.action == "getStartFact")
  {
    auto fact_name = timeline_->actions.getStartFact(params());
    if(fact_name != "")
      res.values.push_back(fact_name);
  }
  else if(req.action == "getEndFact")
  {
    auto fact_name = timeline_->actions.getEndFact(params());
    if(fact_name != "")
      res.values.push_back(fact_name);
  }
  else if(req.action == "getFactsDuring")
    set_res = timeline_->actions.getFactsDuring(params());
  else
    res.code = UNKNOW_ACTION;

  if(res.values.size() == 0)
      set2vector(set_res, res.values);

  return true;
}

bool RosInterface::factHandle(mementar::MementarService::Request &req,
                              mementar::MementarService::Response &res)
{
  res.code = 0;

  removeUselessSpace(req.action);
  removeUselessSpace(req.param);
  param_t params = getParams(req.param);

  std::unordered_set<std::string> set_res;

  if(req.action == "exist")
  {
    if(timeline_->facts.exist(params()))
      res.values.push_back(params());
  }
  else if(req.action == "isActionPart")
  {
    if(timeline_->facts.isActionPart(params()))
      res.values.push_back(params());
  }
  else if(req.action == "getActionPart")
  {
    auto action_name = timeline_->facts.getActionPart(params());
    if(action_name != "")
      res.values.push_back(action_name);
  }
  else if(req.action == "getData")
  {
    auto fact_data = timeline_->facts.getData(params());
    if(fact_data != "")
      res.values.push_back(fact_data);
  }
  else if(req.action == "getStamp")
    res.time_value = ros::Time(timeline_->facts.getStamp(params()));
  else
    res.code = UNKNOW_ACTION;

  if(res.values.size() == 0)
      set2vector(set_res, res.values);

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

void RosInterface::set2string(const std::unordered_set<std::string>& word_set, std::string& result)
{
  for(const std::string& it : word_set)
    result += it + " ";
}

void RosInterface::set2vector(const std::unordered_set<std::string>& word_set, std::vector<std::string>& result)
{
  for(const std::string& it : word_set)
    result.push_back(it);
}

param_t RosInterface::getParams(const std::string& param)
{
  param_t parameters;
  std::vector<std::string> str_params = split(param, " ");

  if(str_params.size())
    parameters.base = str_params[0];

  bool option_found = false;
  for(size_t i = 1; i < str_params.size(); i++)
  {
    /*if((str_params[i] == "-i") || (str_params[i] == "--take_id"))
    {
      i++;
      bool tmp = false;
      if(str_params[i] == "true")
        tmp = true;

      parameters.take_id = tmp;
      option_found = true;
    }
    else if(option_found)
      Display::warning("[WARNING] unknow parameter \"" + str_params[i] + "\"");
    else*/
      parameters.base += " " + str_params[i];
  }

  return parameters;
}

} // namespace mementar
