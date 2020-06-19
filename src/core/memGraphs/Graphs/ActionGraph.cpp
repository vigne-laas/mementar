#include "mementar/core/memGraphs/Graphs/ActionGraph.h"

namespace mementar {

  ActionGraph::ActionGraph(EventGraph* event_graph)
  {
    event_graph_ = event_graph;
  }

  ActionGraph::~ActionGraph()
  {
    for(size_t i = 0; i < all_actions_.size(); i++)
      delete all_actions_[i];
    all_actions_.clear();
  }

  void ActionGraph::add(Action* action)
  {
    all_actions_.push_back(action);
    container_.insert(action);
    event_graph_->add(action->getStartEvent());

    if(action->isPending())
      pending_actions_[action->getName()] = action;
    else
      event_graph_->add(action->getEndEvent());
  }

  bool ActionGraph::setEnd(const std::string& action_name, const SoftPoint& end)
  {
    auto action_it = pending_actions_.find(action_name);
    if(action_it == pending_actions_.end())
      return false;
    else
    {
      if(action_it->second->setEnd(end))
      {
        pending_actions_.erase(action_it);
        event_graph_->add(action_it->second->getEndEvent());
        return true;
      }
      else
        return false;
    }
  }

  std::vector<Action*> ActionGraph::getPending()
  {
    std::vector<Action*> res;
    for(auto act : pending_actions_)
      res.push_back(act.second);
    return res;
  }

} // namespace mementar
