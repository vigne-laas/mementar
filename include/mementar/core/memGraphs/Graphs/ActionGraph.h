#ifndef MEMENTAR_ACTIONGRAPH_H
#define MEMENTAR_ACTIONGRAPH_H

#include "mementar/core/memGraphs/Graphs/Graph.h"
#include "mementar/core/memGraphs/Graphs/EventGraph.h"
#include "mementar/core/memGraphs/Branchs/ContextualizedEvent.h"
#include "mementar/core/memGraphs/Branchs/types/Action.h"

#include <vector>
#include <map>
#include <string>

namespace mementar {

class ActionGraph : public Graph<Action>
{
public:
  ActionGraph(EventGraph* event_graph);
  ~ActionGraph();

  void add(Action* action);
  bool setEnd(const std::string& action_name, const SoftPoint& end);

  std::vector<Action*> get()
  {
    return all_actions_;
  }

  std::vector<Action*> getSafe()
  {
    std::shared_lock<std::shared_timed_mutex> lock(Graph<Action>::mutex_);
    return all_actions_;
  }

  std::vector<Action*> getPending();
  std::vector<Action*> getPendingSafe()
  {
    std::shared_lock<std::shared_timed_mutex> lock(Graph<Action>::mutex_);
    return getPending();
  }


private:
  EventGraph* event_graph_;
  std::vector<Action*> all_actions_;
  std::map<std::string, Action*> pending_actions_;
};

} // namespace mementar

#endif // MEMENTAR_ACTIONGRAPH_H
