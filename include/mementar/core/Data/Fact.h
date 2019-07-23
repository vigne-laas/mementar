#ifndef MEMENTAR_FACT_H
#define MEMENTAR_FACT_H

#include <string>
#include <vector>

#include "mementar/core/Data/StampedData.h"

namespace mementar
{

template<typename T>
class Fact : public StampedData<T>
{
public:
  Fact(T stamp, const std::string& subject, const std::string& predicat, const std::string& object) : StampedData<T>(stamp)
  {
    subject_ = subject;
    predicat_ = predicat;
    object_ = object;
  }

  Fact(T stamp, const std::string& triplet = "") : StampedData<T>(stamp)
  {
    std::vector<std::string> splitted = split(triplet, "|");
    if(splitted.size() >= 1)
      subject_ = splitted[0];
    if(splitted.size() >= 2)
      predicat_ = splitted[1];
    if(splitted.size() >= 3)
      object_ = splitted[2];
  }

  bool valid() const
  {
    return((subject_ != "") && (predicat_ != "") && (object_ != ""));
  }

  std::string toString() const
  {
    return subject_ + "|" + predicat_ + "|" + object_;
  }

  bool operator==(const Fact& other) const
  {
    return ((subject_ == other.subject_)
            && (predicat_ == other.predicat_)
            && (object_ == other.object_));
  }

  bool fit(const Fact& other) const
  {
    return (((subject_ == other.subject_) || (subject_ == "?") || (other.subject_ == "?"))
            && ((predicat_ == other.predicat_) || (predicat_ == "?") || (other.predicat_ == "?"))
            && ((object_ == other.object_) || (object_ == "?") || (other.object_ == "?")));
  }

  friend std::ostream& operator<<(std::ostream& os, const Fact& fact)
  {
    std::string space = " ";
    os << fact.subject_ << space << fact.predicat_ << space << fact.object_;
    return os;
  }

  std::string subject_;
  std::string predicat_;
  std::string object_;

private:

  std::vector<std::string> split(const std::string& str, const std::string& delim)
  {
    std::vector<std::string> tokens;
    size_t prev = 0, pos = 0;
    do
    {
      pos = str.find(delim, prev);
      if (pos == std::string::npos)
        pos = str.length();

      std::string token = str.substr(prev, pos-prev);

      if (!token.empty())
        tokens.push_back(token);
      prev = pos + delim.length();
    }
    while ((pos < str.length()) && (prev < str.length()));

    return tokens;
  }
};

} // namespace mementar

#endif // MEMENTAR_FACT_H
