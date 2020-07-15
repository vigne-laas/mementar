#ifndef MEMENTAR_CONTEXTUALIZEDEVENT_H
#define MEMENTAR_CONTEXTUALIZEDEVENT_H

#include "mementar/core/memGraphs/Branchs/types/Event.h"
#include "mementar/core/memGraphs/Branchs/types/Action.h"
#include "mementar/core/memGraphs/Branchs/ValuedNode.h"

#include "mementar/core/memGraphs/ExtendedBtree/EventLinkedLeaf.h"

#include <string>

namespace mementar {

class ContextualizedEvent : public Event, public ValuedNode, public LinkedEvent<EventLinkedLeaf<SoftPoint::Ttime, ContextualizedEvent*>, ContextualizedEvent>
{
public:
  using LeafType = EventLinkedLeaf<SoftPoint::Ttime, ContextualizedEvent*>;

  ContextualizedEvent(const std::string& id, const Event& other, Action* action = nullptr) : Event(other), ValuedNode(id)
  {
    action_ = action;
  }

  ContextualizedEvent(const std::string& id, const std::string& data, const SoftPoint& soft_point, Action* action = nullptr) : Event(data, soft_point), ValuedNode(id)
  {
    action_ = action;
  }

  virtual ~ContextualizedEvent() {}

  std::string getId() const { return getValue(); }

  bool isPartOfAction() const { return action_ != nullptr; }
  Action* getActionPart() { return action_; }

  std::string toString() const { return getValue() + " " + SoftPoint::toString() + " : " + getData() + std::string(action_ ? " => part of action " + action_->getName() : ""); }

  bool operator==(const ContextualizedEvent& other)
  {
    return this->Event::operator==(other);
  }

  bool isPartOf(const ContextualizedEvent& other)
  {
    return ( (subject_ == other.subject_) &&
             (predicat_ == other.predicat_) );
  }

  friend std::ostream& operator<<(std::ostream& os, const ContextualizedEvent& evt)
  {
    os << evt.toString();
    return os;
  }

  std::vector<ContextualizedEvent*> getNextData()
  {
    std::vector<ContextualizedEvent*> res;
    auto nextLeaf = getNextLeaf();
    if(nextLeaf != nullptr)
    {
      res = nextLeaf->getData();
    }
    return res;
  }

private:
  Action* action_;
};

} // namespace mementar

#endif // MEMENTAR_CONTEXTUALIZEDEVENT_H
