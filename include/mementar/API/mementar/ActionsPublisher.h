#ifndef MEMENTAR_API_ACTIONSPUBLISHER_H
#define MEMENTAR_API_ACTIONSPUBLISHER_H

#include <string>

#include <ros/ros.h>

namespace mementar
{

class ActionsPublisher
{
public:
  ActionsPublisher(ros::NodeHandle* n, const std::string& name = "");

  void insert(const std::string& name, time_t start_stamp = time(0), time_t end_stamp = 0);
  void insertEnd(const std::string& name, time_t end_stamp = time(0));

private:
  ros::NodeHandle* n_;
  ros::Publisher pub_;

  void publish(const std::string& name, time_t start_stamp, time_t end_stamp);
};

} // namespace mementar

#endif // MEMENTAR_API_ACTIONSPUBLISHER_H
