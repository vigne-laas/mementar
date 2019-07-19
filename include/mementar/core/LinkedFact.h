#ifndef MEMENTAR_LINKEDFACT_H
#define MEMENTAR_LINKEDFACT_H

#include <vector>

#include "mementar/core/Fact.h"

namespace mementar
{

class LinkedFact : public Fact
{
public:
  LinkedFact(const std::string& subject, const std::string& predicat, const std::string& object) : Fact(subject, predicat, object)
  {
    next_ = nullptr;
    prev_ = nullptr;
  }

  LinkedFact(const std::string& triplet = "") : Fact(triplet)
  {
    next_ = nullptr;
    prev_ = nullptr;
  }

  bool isEventPart(const LinkedFact& other) const
  {
    return ((subject_ == other.subject_)
            && (predicat_ == other.predicat_));
  }

  LinkedFact* next_;
  LinkedFact* prev_;

  std::vector<LinkedFact*> toLinkNext;
  std::vector<LinkedFact*> toLinkPrev;
};

} // namespace mementar

#endif // MEMENTAR_LINKEDFACT_H
