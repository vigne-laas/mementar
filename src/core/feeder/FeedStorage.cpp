#include "mementar/core/feeder/FeedStorage.h"

#include <iostream>

namespace mementar {

FeedStorage::FeedStorage() : base_regex(R"(^\[(\w+)\](.*)$)")
{
  queue_choice_ = true;
}

void FeedStorage::insert(const std::string& regex, const SoftPoint::Ttime& stamp)
{
  feed_t feed = getFeed(regex, stamp);

  mutex_.lock();
  if(queue_choice_ == true)
    fifo_1.push(feed);
  else
    fifo_2.push(feed);
  mutex_.unlock();
}

void FeedStorage::insert(const std::string& regex, const std::string& expl_regex)
{
  feed_t feed = getFeed(regex);
  feed.expl_ = getFeed(expl_regex).fact_;

  mutex_.lock();
  if(queue_choice_ == true)
    fifo_1.push(feed);
  else
    fifo_2.push(feed);
  mutex_.unlock();
}

void FeedStorage::insert(std::vector<feed_t>& datas)
{
  mutex_.lock();
  if(queue_choice_ == true)
  {
    for(auto& data : datas)
      fifo_1.push(data);
  }
  else
  {
    for(auto& data : datas)
      fifo_2.push(data);
  }
  mutex_.unlock();
}

std::queue<feed_t> FeedStorage::get()
{
  std::queue<feed_t> tmp;
  mutex_.lock();
  if(queue_choice_ == true)
  {
    while(!fifo_2.empty())
      fifo_2.pop();
    queue_choice_ = false;
    tmp = fifo_1;
  }
  else
  {
    while(!fifo_1.empty())
      fifo_1.pop();
    queue_choice_ = true;
    tmp = fifo_2;
  }
  mutex_.unlock();
  return tmp;
}

feed_t FeedStorage::getFeed(const std::string& regex, const SoftPoint::Ttime& stamp)
{
  std::smatch base_match;
  feed_t feed;
  feed.action_ = action_nop;

  if (std::regex_match(regex, base_match, base_regex))
  {
    if (base_match.size() == 3)
    {
      std::string action = base_match[1].str();
      std::transform(action.begin(), action.end(), action.begin(), ::tolower);
      if(action == "add")
        feed.action_ = action_add;
      else if(action == "del")
        feed.action_ = action_del;
      else if(action == "nop")
        return feed;
      else
      {
        std::cout << "data do not match" << std::endl;
        return feed;
      }
    }
  }
  else if(regex == "[nop]")
    return feed;
  else
  {
    std::cout << "data do not match" << std::endl;
    return feed;
  }

  feed.fact_ = Fact(Triplet(base_match[2].str(), feed.action_ == action_add), stamp);
  return feed;
}

} // namespace mementar
