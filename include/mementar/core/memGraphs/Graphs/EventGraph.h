#ifndef MEMENTAR_EVENTGRAPH_H
#define MEMENTAR_EVENTGRAPH_H

#include "mementar/core/memGraphs/Graphs/Graph.h"
#include "mementar/core/memGraphs/Branchs/ContextualizedEvent.h"

#include "mementar/core/memGraphs/ExtendedBtree/EventLinkedLeaf.h"

namespace mementar {

class EventGraph : public Graph<ContextualizedEvent>
{
  using ElBTree = Btree<SoftPoint::Ttime, EventLinkedLeaf<SoftPoint::Ttime, ContextualizedEvent*>, 10>;
public:

  ~EventGraph();

  void add(ContextualizedEvent* event);

  std::vector<ContextualizedEvent*> get()
  {
    return all_events_;
  }

  std::vector<ContextualizedEvent*> getSafe()
  {
    std::shared_lock<std::shared_timed_mutex> lock(Graph<ContextualizedEvent>::mutex_);

    return all_events_;
  }

  ElBTree* getTimeline() { return &timeline; }

private:
  std::vector<ContextualizedEvent*> all_events_;
  ElBTree timeline;
};

} // namespace mementar

#endif // MEMENTAR_EVENTGRAPH_H
