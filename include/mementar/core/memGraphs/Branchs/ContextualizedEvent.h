#ifndef MEMENTAR_CONTEXTUALIZEDEVENT_H
#define MEMENTAR_CONTEXTUALIZEDEVENT_H

#include "mementar/core/memGraphs/Branchs/types/Event.h"
#include "mementar/core/memGraphs/Branchs/types/Action.h"
#include "mementar/core/memGraphs/Branchs/ValuedNode.h"

#include "mementar/core/memGraphs/DoublyLinkedList/DllLinkedElement.h"

#include <string>

namespace mementar {

class ContextualizedEvent : public Event, public ValuedNode, public DllLinkedElement
{
public:
  ContextualizedEvent(const std::string& id, const Event& other, Action* action = nullptr) : Event(other), ValuedNode(id)
  {
    action_ = action;
  }

  ContextualizedEvent(const std::string& id, const std::string& data, const SoftPoint& soft_point, Action* action = nullptr) : Event(data, soft_point), ValuedNode(id)
  {
    action_ = action;
  }

  virtual ~ContextualizedEvent() {}

  std::string getId() { return getValue(); }

  bool isPartOfAction() { return action_ != nullptr; }
  Action* getActionPart() { return action_; }

  std::string toString() { return getValue() + " " + SoftPoint::toString() + " : " + getData() + std::string(action_ ? " is part of action " + action_->getName() : ""); }

  virtual void print(std::ostream& os) const
  {
    os << getValue();
  }

private:
  Action* action_;

  bool operator==(const DllLinkedElement* other)
  {
    return this->operator==(static_cast<const ContextualizedEvent*>(other));
  }
};

} // namespace mementar

#endif // MEMENTAR_CONTEXTUALIZEDEVENT_H
