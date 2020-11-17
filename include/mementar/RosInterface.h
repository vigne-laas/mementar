#ifndef MEMENTAR_ROSINTERFACE_H
#define MEMENTAR_ROSINTERFACE_H

#include <string>
#include <shared_mutex>

#include <ros/ros.h>
#include "std_msgs/String.h"

#include "ontologenius/OntologyManipulator.h"

#include "mementar/MementarAction.h"
#include "mementar/MementarExplanation.h"
#include "mementar/MementarService.h"
#include "mementar/StampedString.h"

//#include "mementar/core/LtManagement/EpisodicTree/ArchivedLeafNode.h"
#include "mementar/core/feeder/Feeder.h"
#include "mementar/core/memGraphs/Timeline.h"
#include "mementar/core/Occasions/OccasionsManager.h"
#include "mementar/core/Parametrization/Configuration.h"

namespace mementar
{

class RosInterface
{
public:
  RosInterface(ros::NodeHandle* n, const std::string& directory, const std::string& configuration_file, size_t order = 10, std::string name = "");
  ~RosInterface();

  void run();
  void stop() {run_ = false; }
  inline bool isRunning() { return run_; }

  void lock();
  void release();

private:
  ros::NodeHandle* n_;
  std::string directory_;
  Configuration configuration_;
  size_t order_;
  OntologyManipulator onto_;

  Timeline* timeline_;
  Feeder feeder_;
  OccasionsManager occasions_;

  std::string name_;
  std::atomic<bool> run_;
  std::shared_timed_mutex mut_;
  std::mutex feeder_mutex_;

  void reset();

  void knowledgeCallback(const std_msgs::String::ConstPtr& msg);
  void stampedKnowledgeCallback(const StampedString::ConstPtr& msg);
  void explanationKnowledgeCallback(const MementarExplanation::ConstPtr& msg);
  void actionKnowledgeCallback(const MementarAction::ConstPtr& msg);

  bool actionsHandle(mementar::MementarService::Request &req,
                     mementar::MementarService::Response &res);

   void feedThread();

  void removeUselessSpace(std::string& text);

  std::string getTopicName(const std::string& topic_name)
  {
    return getTopicName(topic_name, name_);
  }

  std::string getTopicName(const std::string& topic_name, const std::string& onto_name)
  {
    return (onto_name == "") ? "/mementar/" + topic_name : "/mementar/" + topic_name + "/" + onto_name;
  }
};

} // namespace mementar

#endif // MEMENTAR_ROSINTERFACE_H
