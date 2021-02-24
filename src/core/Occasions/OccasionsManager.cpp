#include "mementar/core/Occasions/OccasionsManager.h"

#include "mementar/MementarOccasion.h"

namespace mementar
{

OccasionsManager::OccasionsManager(ros::NodeHandle* n, std::string name) :
                                                                     run_(false),
                                                                     pub_(n->advertise<mementar::MementarOccasion>((name == "") ? "occasions" : "occasions/" + name, 1000))
{
  n_ = n;
  onto_ = nullptr;
  std::string service_name;

  service_name = (name == "") ? "subscribe" : "subscribe/" + name;
  sub_service_ = n_->advertiseService(service_name, &OccasionsManager::SubscribeCallback, this);

  service_name = (name == "") ? "unsubscribe" : "unsubscribe/" + name;
  unsub_service_ = n_->advertiseService(service_name, &OccasionsManager::UnsubscribeCallback, this);
}

OccasionsManager::OccasionsManager(ros::NodeHandle* n, OntologyManipulator* onto, std::string name) :
                                                                     subscription_(onto),
                                                                     run_(false),
                                                                     pub_(n->advertise<mementar::MementarOccasion>((name == "") ? "occasions" : "occasions/" + name, 1000))
{
  n_ = n;
  onto_ = onto;
  std::string service_name;

  service_name = (name == "") ? "subscribe" : "subscribe/" + name;
  sub_service_ = n_->advertiseService(service_name, &OccasionsManager::SubscribeCallback, this);

  service_name = (name == "") ? "unsubscribe" : "unsubscribe/" + name;
  unsub_service_ = n_->advertiseService(service_name, &OccasionsManager::UnsubscribeCallback, this);
}

void OccasionsManager::run()
{
  run_ = true;
  ros::Rate r(50);

  while (ros::ok() && isRunning())
  {
    while(!empty())
    {
      Triplet triplet = get();
      if(triplet.valid())
      {
        std::vector<size_t> ids = subscription_.evaluate(triplet);
        for(auto id : ids)
        {
          mementar::MementarOccasion msg;
          msg.id = id;
          msg.data = triplet.toString();
          msg.last = subscription_.isFinished(id);
          pub_.publish(msg);
        }
      }
    }
    r.sleep();
  }
}

void OccasionsManager::add(const Triplet& triplet)
{
  mutex_.lock();
  if(queue_choice_ == true)
    fifo_1.push(triplet);
  else
    fifo_2.push(triplet);
  mutex_.unlock();
}

bool OccasionsManager::SubscribeCallback(mementar::MementarOccasionSubscription::Request &req,
                                        mementar::MementarOccasionSubscription::Response &res)
{
  TripletPattern triplet_patern = TripletPattern::deserialize(req.data);
  if(!triplet_patern.valid())
    return false;

  res.id = subscription_.subscribe(triplet_patern, req.count);

  return true;
}

bool OccasionsManager::UnsubscribeCallback(mementar::MementarOcassionUnsubscription::Request &req,
                                          mementar::MementarOcassionUnsubscription::Response &res)
{
  if(subscription_.unsubscribe(req.id))
    res.id = req.id;
  else
    res.id = -1;

  return true;
}

Triplet OccasionsManager::get()
{
  Triplet res;
  mutex_.lock();
  if(queue_choice_ == true)
  {
    if(!fifo_2.empty())
    {
      res = fifo_2.front();
      fifo_2.pop();
    }

    if(fifo_2.empty())
      queue_choice_ = false;
  }
  else
  {
    if(!fifo_1.empty())
    {
      res = fifo_1.front();
      fifo_1.pop();
    }

    if(fifo_1.empty())
      queue_choice_ = true;
  }
  mutex_.unlock();
  return res;
}

bool OccasionsManager::empty()
{
  bool res = true;
  mutex_.lock();
  res = fifo_2.empty() && fifo_1.empty();
  mutex_.unlock();
  return res;
}

} // namespace mementar
