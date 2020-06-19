#ifndef MEMENTAR_EVENTGRAPH_H
#define MEMENTAR_EVENTGRAPH_H

#include "mementar/core/memGraphs/Graphs/Graph.h"
#include "mementar/core/memGraphs/Branchs/ContextualizedEvent.h"

#include "mementar/core/memGraphs/Btree/Btree.h"

namespace mementar {

class EventGraph : public Graph<ContextualizedEvent>
{
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

  Btree<SoftPoint::Ttime, DllLinkedElement*, DllCargoNode>* getTimeline() { return &timeline; }

private:
  std::vector<ContextualizedEvent*> all_events_;
  Btree<SoftPoint::Ttime, DllLinkedElement*, DllCargoNode> timeline;
};

} // namespace mementar

#endif // MEMENTAR_EVENTGRAPH_H
