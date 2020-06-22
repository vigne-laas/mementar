#ifndef MEMENTAR_EVENT_H
#define MEMENTAR_EVENT_H

#include <string>
#include <ostream>

#include "mementar/core/memGraphs/Branchs/types/SoftPoint.h"
#include "mementar/core/memGraphs/Branchs/types/Fact.h"

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

  Event(const Fact& fact, const SoftPoint& soft_point) : SoftPoint(soft_point), Fact(fact)
  {
  }


  Event(const Event& other) : SoftPoint(other), Fact(other)
  {
  }

  static std::string serialize(const Event& event)
  {
    if(event.isInstantaneous())
      return "[" + std::to_string(event.getTimeStart()) + "]{" + Fact::serialize(&event) + "}";
    else
      return "[" + std::to_string(event.getTimeStart()) + "," + std::to_string(event.getTimeEnd()) + "]{" + Fact::serialize(&event) + "}";
  }

  static std::string serialize(const Event* event)
  {
    if(event->isInstantaneous())
      return "[" + std::to_string(event->getTimeStart()) + "]{" + Fact::serialize(event) + "}";
    else
      return "[" + std::to_string(event->getTimeStart()) + "," + std::to_string(event->getTimeEnd()) + "]{" + Fact::serialize(event) + "}";
  }

  static Event deserialize(const std::string& str)
  {
    if(std::regex_match(str, match, regex))
    {
      if(match[3].str() == "")
        return Event(Fact::deserialize(match[4].str()), std::stoul(match[1].str()));
      else
        return Event(Fact::deserialize(match[4].str()), SoftPoint(std::stoul(match[1].str()), std::stoul(match[3].str())));
    }
    else
      return Event("", 0);
  }

  static Event* deserializePtr(const std::string& str)
  {
    if(std::regex_match(str, match, regex))
    {
      if(match[3].str() == "")
        return new Event(Fact::deserialize(match[4].str()), std::stoul(match[1].str()));
      else
        return new Event(Fact::deserialize(match[4].str()), SoftPoint(std::stoul(match[1].str()), std::stoul(match[3].str())));

      /*if(match[3].str() == "")
        return new Event(Fact(match[4].str(), match[5].str(), match[6].str(), match[7].str() == "A"), std::stoul(match[1].str()));
      else
        return new Event(Fact(match[4].str(), match[5].str(), match[6].str(), match[7].str() == "A"), SoftPoint(std::stoul(match[1].str()), std::stoul(match[3].str())));*/
    }
    else
      return nullptr;
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

  static std::regex regex;
  static std::smatch match;
};

} // namespace mementar

#endif // MEMENTAR_EVENT_H
