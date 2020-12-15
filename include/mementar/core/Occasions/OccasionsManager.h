#ifndef MEMENTAR_OCCASIONSMANAGER_H
#define MEMENTAR_OCCASIONSMANAGER_H

#include <mutex>
#include <queue>
#include <atomic>

#include <ros/ros.h>

#include "ontologenius/OntologyManipulator.h"

#include "mementar/MementarOccasionSubscription.h"
#include "mementar/MementarOcassionUnsubscription.h"

#include "mementar/core/Occasions/Subscription.h"
#include "mementar/core/memGraphs/Branchs/types/Triplet.h"

namespace mementar
{

class OccasionsManager
{
public:
  OccasionsManager(ros::NodeHandle* n, std::string name = "");
  OccasionsManager(ros::NodeHandle* n, OntologyManipulator* onto, std::string name = "");

  void run();

  void add(const Triplet& triplet);

  void stop() {run_ = false; }
  inline bool isRunning() {return run_; }

private:
  ros::NodeHandle* n_;
  OntologyManipulator* onto_;
  Subscription subscription_;
  std::atomic<bool> run_;

  ros::Publisher pub_;
  ros::ServiceServer sub_service_;
  ros::ServiceServer unsub_service_;

  std::mutex mutex_;

  bool queue_choice_;
  std::queue<Triplet> fifo_1;
  std::queue<Triplet> fifo_2;

  bool SubscribeCallback(mementar::MementarOccasionSubscription::Request &req,
                         mementar::MementarOccasionSubscription::Response &res);
  bool UnsubscribeCallback(mementar::MementarOcassionUnsubscription::Request &req,
                           mementar::MementarOcassionUnsubscription::Response &res);

  Triplet get();
  bool empty();
};

} // namespace mementar

#endif // MEMENTAR_OCCASIONSMANAGER_H
