#include "mementar/core/feeder/Feeder.h"

#include <iostream>

#include "mementar/core/memGraphs/Timeline.h"

namespace mementar {

Feeder::Feeder(Timeline* timeline)// : versionor_(&feed_storage_)
{
  timeline_ = timeline;
}

bool Feeder::run()
{
  if(timeline_ == nullptr)
    return false;

  bool has_run = false;
  std::queue<feed_t> feeds = feed_storage_.get();

  while(feeds.empty() == false)
  {
    has_run = true;
    feed_t feed = feeds.front();
    feeds.pop();

    if((feed.action_ != action_add) && (feed.action_ != action_del))
    {
      /*if(feed.action_ == action_commit)
      {
        if(!versionor_.commit(feed.from_))
          notifications_.push_back("[FAIL][commit]" + feed.from_);
      }
      else if(feed.action_ == action_checkout)
      {
        if(!versionor_.checkout(feed.from_))
          notifications_.push_back("[FAIL][checkout]" + feed.from_);
      }*/
      continue;
    }

    /*if(!feed.checkout_)
      versionor_.insert(feed);*/

    if(feed.fact_.value().valid() == false)
    {
      notifications_.push_back("[FAIL][fact poorly formed]" + feed.fact_.value().Triplet::toString());
    }
    else if(feed.expl_ && (feed.expl_.value().valid() == false))
    {
      notifications_.push_back("[FAIL][explanation poorly formed]" + feed.expl_.value().Triplet::toString());
    }
    else
    {
      if(feed.expl_)
        std::cout << "got explanation : " << feed.fact_.value().Triplet::toString() << " <= " << feed.expl_.value().Triplet::toString() << std::endl;
      else
        std::cout << "got fact : " << feed.fact_.value().Triplet::toString() << std::endl;
    }

  }

  return has_run;
}

} // namespace mementar
