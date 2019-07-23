#ifndef MEMENTAR_LINKEDFACT_H
#define MEMENTAR_LINKEDFACT_H

#include <vector>

#include "mementar/core/Data/Fact.h"

namespace mementar
{

class LinkedFact : public Fact<time_t>
{
public:
  LinkedFact(time_t stamp, const std::string& subject, const std::string& predicat, const std::string& object) : Fact<time_t>(stamp, subject, predicat, object)
  {
    next_ = nullptr;
    prev_ = nullptr;
  }

  LinkedFact(time_t stamp, const std::string& triplet = "") : Fact<time_t>(stamp, triplet)
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
