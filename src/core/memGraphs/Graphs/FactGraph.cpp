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
  all_facts_.push_back(fact);
  container_.insert(fact);
  timeline.insert(fact->getTime(), fact);
}

} // namespace mementar
