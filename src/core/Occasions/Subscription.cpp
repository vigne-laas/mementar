#include "mementar/core/Occasions/Subscription.h"

namespace mementar
{

size_t Subscription::subscribe(const Fact& patern, size_t count)
{
  map_mut_.lock();
  size_t id = id_manager_.getNewId();
  fact_paterns_.insert(std::pair<size_t, Fact>(id, patern));
  counts_[id] = count;
  map_mut_.unlock();

  return id;
}

bool Subscription::unsubscribe(size_t id)
{
  bool res = true;
  map_mut_.lock();
  if(id_manager_.removeId(id))
  {
    fact_paterns_.erase(id);
    counts_.erase(id);
  }
  else
    res = false;

  map_mut_.unlock();
  return res;
}

bool Subscription::isFinished(size_t id)
{
  bool res = true;

  map_mut_.lock();
  if(counts_.find(id) != counts_.end())
    res = (counts_[id] == 0);
  map_mut_.unlock();

  return res;
}

std::vector<size_t> Subscription::evaluate(const Fact& fact)
{
  std::vector<size_t> res;

  map_mut_.lock();
  for(auto it : fact_paterns_)
  {
    if(fact.fit(it.second))
    {
      if(counts_[it.first])
      {
        res.push_back(it.first);
        counts_[it.first]--;
      }
    }
  }
  map_mut_.unlock();

  return res;
}

} // namespace mementar
