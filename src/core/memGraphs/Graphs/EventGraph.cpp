#include "mementar/core/memGraphs/Graphs/EventGraph.h"

namespace mementar {

EventGraph::~EventGraph()
{
  for(size_t i = 0; i < all_events_.size(); i++)
    delete all_events_[i];
  all_events_.clear();
}

void EventGraph::add(ContextualizedEvent* event)
{
  all_events_.push_back(event);
  container_.insert(event);
  timeline.insert(event->getTime(), event);
}

} // namespace mementar
