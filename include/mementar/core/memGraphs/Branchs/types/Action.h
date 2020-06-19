#ifndef MEMENTAR_ACTION_H
#define MEMENTAR_ACTION_H

#include "mementar/core/memGraphs/Branchs/ValuedNode.h"
#include "mementar/core/memGraphs/Branchs/types/SoftPoint.h"

#include <experimental/optional>

namespace mementar {

class ContextualizedEvent;

class Action : public ValuedNode
{
public:
  Action(const std::string& name, const SoftPoint& start, const std::experimental::optional<SoftPoint>& end = std::experimental::nullopt);
  Action(const std::string& name, const SoftPoint& start, const SoftPoint::Ttime& end);

  bool setEnd(const SoftPoint& end);

  std::string getName() { return getValue(); }

  size_t getDuration();
  size_t getMinDuration();
  size_t getMaxDuration();

  bool isSoft();
  bool isPending() { return !end_; }

  ContextualizedEvent* getStartEvent() { return start_; }
  ContextualizedEvent* getEndEvent() { return end_.value(); }

private:
  ContextualizedEvent* start_;
  std::experimental::optional<ContextualizedEvent*> end_;
};

} // namespace mementar

#endif // MEMENTAR_ACTION_H
