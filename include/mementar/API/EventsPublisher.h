#ifndef MEMENTAR_EVENTSPUBLISHER_H
#define MEMENTAR_EVENTSPUBLISHER_H

#include <string>

#include <ros/ros.h>

namespace mementar
{

class EventsPublisher
{
public:
  EventsPublisher(ros::NodeHandle* n, const std::string& name = "");

  void insert(const std::string& subject, const std::string& predicat, const std::string& object, time_t stamp = 0);
  void insert(const std::string& data, time_t stamp = 0);

private:
  ros::NodeHandle* n_;
  ros::Publisher pub_;

  void publish(const std::string& str, time_t stamp = 0);
};

} // namespace mementar

#endif // MEMENTAR_EVENTSPUBLISHER_H
