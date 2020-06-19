#ifndef MEMENTAR_EVENTSMANAGER_H
#define MEMENTAR_EVENTSMANAGER_H

#include <mutex>
#include <queue>
#include <atomic>

#include <ros/ros.h>

#include "mementar/MementarEventSubscription.h"
#include "mementar/MementarEventUnsubscription.h"

#include "mementar/core/Events/Subscription.h"
#include "mementar/core/memGraphs/Fact.h"

namespace mementar
{

class EventsManager
{
public:
  EventsManager(ros::NodeHandle* n, std::string name = "");

  void run();

  void add(const Fact& fact);

  void stop() {run_ = false; }
  inline bool isRunning() {return run_; }

private:
  ros::NodeHandle* n_;
  Subscription subscription_;
  std::atomic<bool> run_;

  ros::Publisher pub_;
  ros::ServiceServer sub_service_;
  ros::ServiceServer unsub_service_;

  std::mutex mutex_;

  bool queue_choice_;
  std::queue<Fact> fifo_1;
  std::queue<Fact> fifo_2;

  bool SubscribeCallback(mementar::MementarEventSubscription::Request &req,
                         mementar::MementarEventSubscription::Response &res);
  bool UnsubscribeCallback(mementar::MementarEventUnsubscription::Request &req,
                           mementar::MementarEventUnsubscription::Response &res);

  Fact get();
  bool empty();
};

} // namespace mementar

#endif // MEMENTAR_EVENTSMANAGER_H
