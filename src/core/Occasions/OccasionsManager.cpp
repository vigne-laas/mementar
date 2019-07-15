#include "mementar/core/Occasions/OccasionsManager.h"

#include "mementar/MementarOccasion.h"

namespace mementar
{

OccasionsManager::OccasionsManager(ros::NodeHandle* n, std::string name) : run_(false),
                                                                     pub_(n->advertise<mementar::MementarOccasion>((name == "") ? "occasions" : "occasions/" + name, 1000))
{
  n_ = n;
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
      Fact fact = get();
      if(fact.valid())
      {
        std::vector<size_t> ids = subscription_.evaluate(fact);
        for(auto id : ids)
        {
          mementar::MementarOccasion msg;
          msg.id = id;
          msg.data = fact.toString();
          msg.last = subscription_.isFinished(id);
          pub_.publish(msg);
        }
      }
    }
    r.sleep();
  }
}

void OccasionsManager::add(const Fact& fact)
{
  mutex_.lock();
  if(queue_choice_ == true)
    fifo_1.push(fact);
  else
    fifo_2.push(fact);
  mutex_.unlock();
}

bool OccasionsManager::SubscribeCallback(mementar::MementarOccasionSubscription::Request &req,
                                        mementar::MementarOccasionSubscription::Response &res)
{
  Fact fact_patern(req.data);
  if(!fact_patern.valid())
    return false;

  res.id = subscription_.subscribe(fact_patern, req.count);

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

Fact OccasionsManager::get()
{
  Fact res;
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
