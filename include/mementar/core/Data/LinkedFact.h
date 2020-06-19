#ifndef MEMENTAR_LINKEDFACT_H
#define MEMENTAR_LINKEDFACT_H

#include <vector>

#include "mementar/core/memGraphs/Branchs/types/Fact.h"

namespace mementar
{

template<typename T>
class LinkedFact : public Fact<T>
{
public:
  LinkedFact(T stamp, const std::string& subject, const std::string& predicat, const std::string& object) : Fact<T>(stamp, subject, predicat, object)
  {
    next_ = nullptr;
    prev_ = nullptr;
  }

  LinkedFact(T stamp, const std::string& triplet = "") : Fact<T>(stamp, triplet)
  {
    next_ = nullptr;
    prev_ = nullptr;
  }

  bool isEventPart(const LinkedFact<T>& other) const
  {
    return ((this->subject_ == other.subject_)
            && (this->predicat_ == other.predicat_));
  }

  LinkedFact<T>* next_;
  LinkedFact<T>* prev_;

  std::vector<LinkedFact<T>*> toLinkNext;
  std::vector<LinkedFact<T>*> toLinkPrev;
};

} // namespace mementar

#endif // MEMENTAR_LINKEDFACT_H
