#ifndef MEMENTAR_SUBSCRIPTION_H
#define MEMENTAR_SUBSCRIPTION_H

#include <map>
#include <vector>
#include <mutex>

#include "mementar/core/memGraphs/Fact.h"
#include "mementar/core/Events/IdManager.h"

namespace mementar
{

class Subscription
{
public:
  size_t subscribe(const Fact& patern, size_t count);
  bool unsubscribe(size_t id);

  bool isFinished(size_t id);
  bool empty() { return fact_paterns_.size() == 0; }

  std::vector<size_t> evaluate(const Fact& fact);

private:
  std::map<size_t, Fact> fact_paterns_;
  std::map<size_t, size_t> counts_;
  std::mutex map_mut_;

  IdManager<size_t> id_manager_;
};

} // namespace mementar

#endif // MEMENTAR_SUBSCRIPTION_H
