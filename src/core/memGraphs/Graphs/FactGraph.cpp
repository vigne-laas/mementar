#include "mementar/core/memGraphs/Graphs/FactGraph.h"

namespace mementar {

FactGraph::~FactGraph()
{
  for(size_t i = 0; i < all_facts_.size(); i++)
    delete all_facts_[i];
  all_facts_.clear();
}

void FactGraph::add(ContextualizedFact* fact)
{
  std::cout << "ADD fact " << fact->toString() << std::endl;
  all_facts_.push_back(fact);
  container_.insert(fact);
  timeline_.insert(fact->getTime(), fact);
}

ContextualizedFact* FactGraph::findRecent(const Triplet& triplet, SoftPoint::Ttime until)
{
  for(BplusLeaf<SoftPoint::Ttime, ContextualizedFact*>* leaf = timeline_.getLast(); leaf != nullptr; leaf = leaf->getPreviousLeaf())
  {
    for(auto data : leaf->payload_)
    {
      if(data->getTime() < until)
        return nullptr;
      else if(data->fit(triplet))
      {
        return data;
      }
    }
  }
  return nullptr;
}

} // namespace mementar
