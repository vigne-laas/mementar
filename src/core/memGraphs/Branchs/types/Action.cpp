#include "mementar/core/memGraphs/Branchs/types/Action.h"

#include "mementar/core/memGraphs/Branchs/ContextualizedEvent.h"

namespace mementar {

Action::Action(const std::string& name, const SoftPoint& start, const std::experimental::optional<SoftPoint>& end) : ValuedNode(name)
{
  start_ = new ContextualizedEvent(std::string(name + "_start"), Event(name + "|_|start", start), this);
  if(end)
    end_ = new ContextualizedEvent(std::string(name + "_end"), Event(name + "|_|end", end.value()), this);
}

Action::Action(const std::string& name, const SoftPoint& start, const SoftPoint::Ttime& end) : ValuedNode(name)
{
  start_ = new ContextualizedEvent(std::string(name + "_start"), Event(name + "|_|start", start), this);
  end_ = new ContextualizedEvent(std::string(name + "_end"), Event(name + "|_|end", end), this);
}

bool Action::setEnd(const SoftPoint& end)
{
  if(end_)
    return false;

  end_ = new ContextualizedEvent(std::string(getValue() + "_end"), Event(getValue() + "|_|end", end), this);
  return true;
}

size_t Action::getDuration()
{
  return (end_ ? end_.value()->getTime() - start_->getTime() : 0);
}

size_t Action::getMinDuration()
{
  return (end_ ? end_.value()->getTimeStart() - start_->getTimeEnd() : 0);
}

size_t Action::getMaxDuration()
{
  return (end_ ? end_.value()->getTimeEnd() - start_->getTimeStart() : 0);
}

bool Action::isSoft()
{
  if(end_)
    return ((start_->isInstantaneous() && end_.value()->isInstantaneous()) == false);
  else
    return (start_->isInstantaneous() == false);
}

} // namespace mementar
