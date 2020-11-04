#include "mementar/API/OccasionsSubscriber.h"

#include "mementar/MementarOccasionSubscription.h"
#include "mementar/MementarOcassionUnsubscription.h"

namespace mementar
{

OccasionsSubscriber::OccasionsSubscriber(std::function<void(const Fact&)> callback, const std::string& name, bool spin_thread)
{
  callback_ = callback;

  if(spin_thread)
  {
    need_to_terminate_ = false;
    n_.setCallbackQueue(&callback_queue_);
    spin_thread_ = new std::thread(std::bind(&OccasionsSubscriber::spinThread, this));
  }
  else
    spin_thread_ = nullptr;

  sub_ = n_.subscribe((name == "") ? "mementar/occasions" : "mementar/occasions/" + name, 1000, &OccasionsSubscriber::occasionCallback, this);
  client_subscribe_ = n_.serviceClient<MementarOccasionSubscription>((name == "") ? "mementar/subscribe" : "mementar/subscribe/" + name);
  client_cancel_ = n_.serviceClient<MementarOcassionUnsubscription>((name == "") ? "mementar/unsubscribe" : "mementar/unsubscribe/" + name);
}

OccasionsSubscriber::OccasionsSubscriber(std::function<void(const Fact&)> callback, bool spin_thread)
{
  callback_ = callback;

  if(spin_thread)
  {
    need_to_terminate_ = false;
    n_.setCallbackQueue(&callback_queue_);
    spin_thread_ = new std::thread(std::bind(&OccasionsSubscriber::spinThread, this));
  }
  else
    spin_thread_ = nullptr;

  sub_ = n_.subscribe("mementar/occasions", 1000, &OccasionsSubscriber::occasionCallback, this);
  client_subscribe_ = n_.serviceClient<MementarOccasionSubscription>("mementar/subscribe");
  client_cancel_ = n_.serviceClient<MementarOcassionUnsubscription>("mementar/unsubscribe");
}

OccasionsSubscriber::~OccasionsSubscriber()
{
  cancel();
  sub_.shutdown();
  if(spin_thread_)
  {
    need_to_terminate_ = true;
    spin_thread_->join();
    delete spin_thread_;
  }
}

bool OccasionsSubscriber::subscribe(const Fact& pattern, size_t count)
{
  MementarOccasionSubscription srv;
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

bool OccasionsSubscriber::cancel()
{
  bool done = true;
  for(size_t i = 0; i < ids_.size();)
  {
    MementarOcassionUnsubscription srv;
    srv.request.id = ids_[i];
    if(client_cancel_.call(srv))
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

void OccasionsSubscriber::occasionCallback(MementarOccasion msg)
{
  auto it = std::find(ids_.begin(), ids_.end(), msg.id);
  if(it != ids_.end())
  {
    callback_(Fact(msg.data));
    if(msg.last == true)
      ids_.erase(it);
  }
}

void OccasionsSubscriber::spinThread()
{
  while(n_.ok())
  {
    if(need_to_terminate_)
      break;
    callback_queue_.callAvailable(ros::WallDuration(0.1));
  }
}

} // namespace mementar
