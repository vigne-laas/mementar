#include "mementar/API/EventsSubscriber.h"

#include "mementar/MementarEventSubscription.h"
#include "mementar/MementarEventUnsubscription.h"

namespace mementar
{

EventsSubscriber::EventsSubscriber(ros::NodeHandle* n, std::function<void(const Event&)> callback, const std::string& name)
{
  n_ = n;
  callback_ = callback;

  sub_ = n_->subscribe((name == "") ? "mementar/events" : "mementar/events/" + name, 1000, &EventsSubscriber::eventCallback, this);
  client_subscribe_ = n_->serviceClient<MementarEventSubscription>((name == "") ? "mementar/subscribe" : "mementar/subscribe/" + name);
  client_cancel_ = n_->serviceClient<MementarEventUnsubscription>((name == "") ? "mementar/unsubscribe" : "mementar/unsubscribe/" + name);
}

bool EventsSubscriber::subscribe(const Event& patern, size_t count)
{
  MementarEventSubscription srv;
  srv.request.data = patern();
  srv.request.count = count;

  if(client_subscribe_.call(srv))
  {
    ids_.push_back(srv.response.id);
    return true;
  }
  else
    return false;
}

bool EventsSubscriber::cancel()
{
  bool done = true;
  for(size_t i = 0; i < ids_.size();)
  {
    MementarEventUnsubscription srv;
    srv.request.id = ids_[i];
    if(client_subscribe_.call(srv))
    {
      if(srv.response.id != (int)ids_[i])
        done = false;
    }
    else
      done = false;

    if(done)
      ids_.erase(ids_.begin() + i);
    else
      i++;
  }

  return done;
}

void EventsSubscriber::eventCallback(MementarEvent msg)
{
  auto it = std::find(ids_.begin(), ids_.end(), msg.id);
  if(it != ids_.end())
  {
    callback_(Event(msg.data));
    if(msg.last == true)
      ids_.erase(it);
  }
}

} // namespace mementar
