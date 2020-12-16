#ifndef MEMENTAR_FACTGRAPH_H
#define MEMENTAR_FACTGRAPH_H

#include "mementar/core/memGraphs/Graphs/Graph.h"
#include "mementar/core/memGraphs/Branchs/ContextualizedFact.h"

#include "mementar/core/memGraphs/ExtendedBtree/EventLinkedLeaf.h"

namespace mementar {

class FactGraph : public Graph<ContextualizedFact>
{
  using ElBTree = Btree<SoftPoint::Ttime, EventLinkedLeaf<SoftPoint::Ttime, ContextualizedFact*>, 10>;
public:

  ~FactGraph();

  void add(ContextualizedFact* fact);

  bool exist(const std::string& fact_id);
  bool isActionPart(const std::string& fact_id);
  std::string getActionPart(const std::string& fact_id);
  std::string getData(const std::string& fact_id);

  std::vector<ContextualizedFact*> get()
  {
    return all_facts_;
  }

  std::vector<ContextualizedFact*> getSafe()
  {
    std::shared_lock<std::shared_timed_mutex> lock(Graph<ContextualizedFact>::mutex_);

    return all_facts_;
  }

  ElBTree* getTimeline() { return &timeline_; }

  ContextualizedFact* find(const std::string& fact_id);
  ContextualizedFact* findRecent(const Triplet& triplet, SoftPoint::Ttime until = SoftPoint::default_time);

private:
  std::vector<ContextualizedFact*> all_facts_;
  ElBTree timeline_;
};

} // namespace mementar

#endif // MEMENTAR_FACTGRAPH_H
