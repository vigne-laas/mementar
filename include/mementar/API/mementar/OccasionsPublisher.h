#ifndef MEMENTAR_API_OCCASIONSPUBLISHER_H
#define MEMENTAR_API_OCCASIONSPUBLISHER_H

#include <string>

#include <ros/ros.h>

#include "mementar/API/mementar/Fact.h"

namespace mementar
{

class OccasionsPublisher
{
public:
  OccasionsPublisher(ros::NodeHandle* n, const std::string& name = "");

  void insert(const Fact& fact, time_t stamp = time(0));

private:
  ros::NodeHandle* n_;
  ros::Publisher pub_;

  void publish(const std::string& str, time_t stamp = time(0));
};

} // namespace mementar

#endif // MEMENTAR_API_OCCASIONSPUBLISHER_H
