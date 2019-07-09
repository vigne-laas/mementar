#ifndef MEMENTAR_EVENTSSUBSCRIBER_H
#define MEMENTAR_EVENTSSUBSCRIBER_H

#include <string>
#include <vector>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "mementar/MementarEvent.h"
#include "mementar/API/Event.h"

namespace mementar
{

class EventsSubscriber
{
public:
  EventsSubscriber(std::function<void(const Event&)> callback, const std::string& name = "", bool spin_thread = true);
  EventsSubscriber(std::function<void(const Event&)> callback, bool spin_thread);
  ~EventsSubscriber() { cancel(); }

  bool subscribe(const Event& pattern, size_t count = -1);
  bool cancel();

  bool end() { return ids_.size() == 0; }

private:
  ros::NodeHandle n_;
  ros::Subscriber sub_;
  ros::ServiceClient client_subscribe_;
  ros::ServiceClient client_cancel_;

  std::mutex terminate_mutex_;
  bool need_to_terminate_;
  std::thread* spin_thread_;
  ros::CallbackQueue callback_queue_;

  std::vector<size_t> ids_;

  void eventCallback(MementarEvent msg);

  std::function<void(const Event&)> callback_;

  void spinThread();
};

} // namespace mementar

#endif // MEMENTAR_EVENTSSUBSCRIBER_H
