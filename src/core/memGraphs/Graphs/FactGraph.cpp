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

bool FactGraph::exist(const std::string& fact_id)
{
  return (find(fact_id) != nullptr);
}

bool FactGraph::isActionPart(const std::string& fact_id)
{
  auto fact = find(fact_id);
  if(fact == nullptr)
    return false;
  else
    return fact->isPartOfAction();
}

std::string FactGraph::getActionPart(const std::string& fact_id)
{
  auto fact = find(fact_id);
  if(fact == nullptr)
    return "";
  else if(fact->isPartOfAction())
    return fact->getActionPart()->getValue();
  else
    return "";
}

std::string FactGraph::getData(const std::string& fact_id)
{
  auto fact = find(fact_id);
  if(fact == nullptr)
    return "";
  else
    return fact->getData();
}

ContextualizedFact* FactGraph::find(const std::string& fact_id)
{
  return container_.find(fact_id);
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
