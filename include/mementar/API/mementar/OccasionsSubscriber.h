#ifndef MEMENTAR_API_OCCASIONSSUBSCRIBER_H
#define MEMENTAR_API_OCCASIONSSUBSCRIBER_H

#include <string>
#include <vector>
#include <thread>
#include <atomic>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "mementar/MementarOccasion.h"
#include "mementar/API/mementar/Fact.h"

namespace mementar
{

class OccasionsSubscriber
{
public:
  OccasionsSubscriber(std::function<void(const Fact&)> callback, const std::string& name = "", bool spin_thread = true);
  OccasionsSubscriber(std::function<void(const Fact&)> callback, bool spin_thread);
  virtual ~OccasionsSubscriber();

  bool subscribe(const Fact& pattern, size_t count = -1);
  bool cancel();

  bool end() { return ids_.size() == 0; }

private:
  ros::Subscriber sub_;
  ros::ServiceClient client_subscribe_;
  ros::ServiceClient client_cancel_;

  std::atomic<bool> need_to_terminate_;
  std::thread* spin_thread_;
  ros::CallbackQueue callback_queue_;
  ros::NodeHandle n_;

  std::vector<size_t> ids_;

  void occasionCallback(MementarOccasion msg);

  std::function<void(const Fact&)> callback_;

  void spinThread();
};

} // namespace mementar

#endif // MEMENTAR_API_OCCASIONSSUBSCRIBER_H
