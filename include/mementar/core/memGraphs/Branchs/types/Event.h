#ifndef MEMENTAR_EVENT_H
#define MEMENTAR_EVENT_H

#include <string>
#include <ostream>

#include "mementar/core/memGraphs/Branchs/types/SoftPoint.h"
#include "mementar/core/memGraphs/Fact.h"

namespace mementar {

class Event : public SoftPoint, public Fact
{
public:
  Event(const std::string& data, size_t t_start, std::experimental::optional<size_t> t_end = std::experimental::nullopt) : SoftPoint(t_start, t_end), Fact(data)
  {
  }

  Event(const std::string& data, const SoftPoint& soft_point) : SoftPoint(soft_point), Fact(data)
  {
  }

  Event(const Event& other) : SoftPoint(other), Fact(other)
  {
  }

  std::string getData() { return Fact::toString(); }

  friend std::ostream& operator<<(std::ostream& os, Event* event)
  {
    std::string space = " ";
    os << event->getData();
    return os;
  }

  bool operator==(const Event& other)
  {
    return Fact::operator==(other);
  }

  bool operator==(const Event* other)
  {
    return Fact::operator==(other);
  }

protected:
};

} // namespace mementar

#endif // MEMENTAR_EVENT_H
