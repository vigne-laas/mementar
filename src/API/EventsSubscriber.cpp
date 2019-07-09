#include "mementar/API/EventsSubscriber.h"

#include "mementar/MementarEventSubscription.h"
#include "mementar/MementarEventUnsubscription.h"

namespace mementar
{

EventsSubscriber::EventsSubscriber(std::function<void(const Event&)> callback, const std::string& name, bool spin_thread)
{
  callback_ = callback;

  sub_ = n_.subscribe((name == "") ? "mementar/events" : "mementar/events/" + name, 1000, &EventsSubscriber::eventCallback, this);
  client_subscribe_ = n_.serviceClient<MementarEventSubscription>((name == "") ? "mementar/subscribe" : "mementar/subscribe/" + name);
  client_cancel_ = n_.serviceClient<MementarEventUnsubscription>((name == "") ? "mementar/unsubscribe" : "mementar/unsubscribe/" + name);

  if(spin_thread)
  {
    need_to_terminate_ = false;
    n_.setCallbackQueue(&callback_queue_);
    spin_thread_ = new std::thread(std::bind(&EventsSubscriber::spinThread, this));
  }
  else
    spin_thread_ = nullptr;
}

EventsSubscriber::EventsSubscriber(std::function<void(const Event&)> callback, bool spin_thread)
{
  callback_ = callback;

  sub_ = n_.subscribe("mementar/events", 1000, &EventsSubscriber::eventCallback, this);
  client_subscribe_ = n_.serviceClient<MementarEventSubscription>("mementar/subscribe");
  client_cancel_ = n_.serviceClient<MementarEventUnsubscription>("mementar/unsubscribe");

  if(spin_thread)
  {
    need_to_terminate_ = false;
    n_.setCallbackQueue(&callback_queue_);
    spin_thread_ = new std::thread(std::bind(&EventsSubscriber::spinThread, this));
  }
  else
    spin_thread_ = nullptr;
}

EventsSubscriber::~EventsSubscriber()
{
  cancel();
  if(spin_thread_)
  {
    terminate_mutex_.lock();
    need_to_terminate_ = true;
    terminate_mutex_.unlock();
    spin_thread_->join();
    delete spin_thread_;
  }
}

bool EventsSubscriber::subscribe(const Event& pattern, size_t count)
{
  MementarEventSubscription srv;
  srv.request.data = pattern();
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

void EventsSubscriber::spinThread()
{
  while(n_.ok())
  {
    terminate_mutex_.lock();
    if(need_to_terminate_)
      break;
    terminate_mutex_.unlock();
    callback_queue_.callAvailable(ros::WallDuration(0.1));
  }
}

} // namespace mementar
