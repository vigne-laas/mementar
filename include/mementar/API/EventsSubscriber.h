#ifndef MEMENTAR_EVENTSSUBSCRIBER_H
#define MEMENTAR_EVENTSSUBSCRIBER_H

#include <string>
#include <vector>

#include <ros/ros.h>

#include "mementar/MementarEvent.h"
#include "mementar/API/Event.h"

namespace mementar
{

class EventsSubscriber
{
public:
  EventsSubscriber(ros::NodeHandle* n, std::function<void(const Event&)> callback, const std::string& name = "");

  bool subscribe(const Event& patern, size_t count = -1);
  bool cancel();

  bool end() { return ids_.size() == 0; }

private:
  ros::NodeHandle* n_;
  ros::Subscriber sub_;
  ros::ServiceClient client_subscribe_;
  ros::ServiceClient client_cancel_;

  std::vector<size_t> ids_;

  void eventCallback(MementarEvent msg);

  std::function<void(const Event&)> callback_;
};

} // namespace mementar

#endif // MEMENTAR_EVENTSSUBSCRIBER_H
