#ifndef MEMENTAR_ROSINTERFACE_H
#define MEMENTAR_ROSINTERFACE_H

#include <string>

#include <ros/ros.h>
#include "std_msgs/String.h"

#include "mementar/MementarService.h"
#include "mementar/StampedString.h"

#include "mementar/core/EpisodicTree/ArchivedLeafNode.h"

namespace mementar
{

class RosInterface
{
public:
  RosInterface(ros::NodeHandle* n, const std::string& directory, size_t order = 10, std::string name = "");
  ~RosInterface();

  void run();
  void stop() {run_ = false; }
  inline bool isRunning() {return run_; }
  ArchivedLeafNode* getTree() {return tree_; }

private:
  ros::NodeHandle* n_;
  ArchivedLeafNode* tree_;

  std::string name_;
  std::atomic<bool> run_;

  void knowledgeCallback(const std_msgs::String::ConstPtr& msg);
  void stampedKnowledgeCallback(const StampedString::ConstPtr& msg);

  bool actionsHandle(mementar::MementarService::Request &req,
                     mementar::MementarService::Response &res);

  void removeUselessSpace(std::string& text);
};

} // namespace mementar

#endif // MEMENTAR_ROSINTERFACE_H
