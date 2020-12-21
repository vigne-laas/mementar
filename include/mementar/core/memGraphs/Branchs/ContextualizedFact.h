#ifndef MEMENTAR_CONTEXTUALIZEDFACT_H
#define MEMENTAR_CONTEXTUALIZEDFACT_H

#include "mementar/core/memGraphs/Branchs/types/Fact.h"
#include "mementar/core/memGraphs/Branchs/types/Action.h"
#include "mementar/core/memGraphs/Branchs/ValuedNode.h"

#include "mementar/core/memGraphs/ExtendedBtree/EventLinkedLeaf.h"

#include <string>

namespace mementar {

class ContextualizedFact : public Fact, public ValuedNode, public LinkedEvent<EventLinkedLeaf<SoftPoint::Ttime, ContextualizedFact*>, ContextualizedFact>
{
public:
  using LeafType = EventLinkedLeaf<SoftPoint::Ttime, ContextualizedFact*>;

  ContextualizedFact(const std::string& id, const Fact& other, Action* action = nullptr) : Fact(other), ValuedNode(id)
  {
    action_ = action;
  }

  ContextualizedFact(const std::string& id, const std::string& data, const SoftPoint& soft_point, Action* action = nullptr) : Fact(data, soft_point), ValuedNode(id)
  {
    action_ = action;
  }

  virtual ~ContextualizedFact() {}

  std::string getId() const { return getValue(); }

  bool isPartOfAction() const { return action_ != nullptr; }
  Action* getActionPart() { return action_; }

  std::string toString() const { return getValue() + " " + SoftPoint::toString() + " : " + getData() + std::string(action_ ? " => part of action " + action_->getName() : ""); }

  bool operator==(const ContextualizedFact& other)
  {
    return this->Fact::operator==(other);
  }

  bool isPartOf(const ContextualizedFact& other)
  {
    return ( (subject_ == other.subject_) &&
             (predicat_ == other.predicat_) );
  }

  friend std::ostream& operator<<(std::ostream& os, const ContextualizedFact& ctx_fact)
  {
    os << ctx_fact.toString();
    return os;
  }

  std::vector<ContextualizedFact*> getNextData()
  {
    std::vector<ContextualizedFact*> res;
    auto next_leaf = getNextLeaf();
    if(next_leaf != nullptr)
      res = next_leaf->getData();

    return res;
  }

  std::vector<ContextualizedFact*> getPreviousData()
  {
    std::vector<ContextualizedFact*> res;
    auto prev_leaf = getPreviousLeaf();
    if(prev_leaf != nullptr)
      res = prev_leaf->getData();

    return res;
  }

private:
  Action* action_;
};

} // namespace mementar

#endif // MEMENTAR_CONTEXTUALIZEDFACT_H
