#include "mementar/core/feeder/FeedStorage.h"

#include <iostream>

namespace mementar {

FeedStorage::FeedStorage() : base_regex(R"(^\[(\w+)\](.*)$)")
{

}

void FeedStorage::insertFact(const std::string& regex, const SoftPoint::Ttime& stamp)
{
  feed_fact_t feed = getFeedFact(regex, stamp);
  fact_queue_.insert(feed);
}

void FeedStorage::insertFact(const std::string& regex, const std::string& expl_regex)
{
  feed_fact_t feed = getFeedFact(regex);
  feed.expl_ = getFeedFact(expl_regex).fact_;
  fact_queue_.insert(feed);
}

void FeedStorage::insertFacts(std::vector<feed_fact_t>& datas)
{
  fact_queue_.insert(datas);
}

void FeedStorage::insertAction(const std::string& name, const SoftPoint::Ttime& start_stamp, const SoftPoint::Ttime& end_stamp)
{
  feed_action_t feed;
  feed.name_ = name;
  feed.t_start_ = start_stamp;
  feed.t_end_ = end_stamp;
  action_queue_.insert(feed);
}

void FeedStorage::insertActions(std::vector<feed_action_t>& datas)
{
  action_queue_.insert(datas);
}

std::queue<feed_fact_t> FeedStorage::getFacts()
{
  return fact_queue_.get();
}

std::queue<feed_action_t> FeedStorage::getActions()
{
  return action_queue_.get();
}

feed_fact_t FeedStorage::getFeedFact(const std::string& regex, const SoftPoint::Ttime& stamp)
{
  std::smatch base_match;
  feed_fact_t feed;
  feed.cmd_ = cmd_nop;

  if (std::regex_match(regex, base_match, base_regex))
  {
    if (base_match.size() == 3)
    {
      std::string action = base_match[1].str();
      std::transform(action.begin(), action.end(), action.begin(), ::tolower);
      if(action == "add")
        feed.cmd_ = cmd_add;
      else if(action == "del")
        feed.cmd_ = cmd_del;
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

  feed.fact_ = Fact(Triplet(base_match[2].str(), feed.cmd_ == cmd_add), stamp);
  return feed;
}

} // namespace mementar
