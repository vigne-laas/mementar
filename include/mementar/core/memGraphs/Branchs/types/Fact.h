#ifndef MEMENTAR_FACT_H
#define MEMENTAR_FACT_H

#include <string>
#include <vector>
#include <regex>
#include <iostream>

namespace mementar
{

class Fact
{
public:
  Fact(const std::string& subject, const std::string& predicat, const std::string& object, bool add = true)
  {
    subject_ = subject;
    predicat_ = predicat;
    object_ = object;
    add_ = add;
  }

  Fact(const std::string& triplet = "", bool add = true)
  {
    std::vector<std::string> splitted = split(triplet, "|");
    if(splitted.size() >= 1)
      subject_ = splitted[0];
    if(splitted.size() >= 2)
      predicat_ = splitted[1];
    if(splitted.size() >= 3)
      object_ = splitted[2];
    add_ = add;
  }

  Fact(const Fact& other)
  {
    subject_ = other.subject_;
    predicat_ = other.predicat_;
    object_ = other.object_;
    add_ = other.add_;
  }

  static std::string serialize(const Fact& fact)
  {
    return (fact.add_ ? "A|" : "D|") + fact.subject_ + "|" + fact.predicat_ + "|" + fact.object_;
  }

  static std::string serialize(const Fact* fact)
  {
    return (fact->add_ ? "A|" : "D|") + fact->subject_ + "|" + fact->predicat_ + "|" + fact->object_;
  }

  static Fact deserialize(const std::string& str)
  {
    if(std::regex_match(str, match, regex))
      return Fact(match[2].str(), match[3].str(), match[4].str(), match[1].str() == "A");
    else
      return Fact();
  }

  static Fact* deserializePtr(const std::string& str)
  {
    if(std::regex_match(str, match, regex))
      return new Fact(match[2].str(), match[3].str(), match[4].str(), match[1].str() == "A");
    else
      return nullptr;
  }

  bool valid() const
  {
    return((subject_ != "") && (predicat_ != "") && (object_ != ""));
  }

  std::string toString() const
  {
    return (add_ ? "[add]" : "[del]") + subject_ + " | " + predicat_ + " | " + object_;
  }

  bool operator==(const Fact& other) const
  {
    return ( (add_ == other.add_)
            && (subject_ == other.subject_)
            && (predicat_ == other.predicat_)
            && (object_ == other.object_));
  }

  bool operator==(const Fact* other) const
  {
    return ( (add_ == other->add_)
            && (subject_ == other->subject_)
            && (predicat_ == other->predicat_)
            && (object_ == other->object_));
  }

  bool fit(const Fact& other) const
  {
    return ( (add_ == other.add_)
            && ((subject_ == other.subject_) || (subject_ == "?") || (other.subject_ == "?"))
            && ((predicat_ == other.predicat_) || (predicat_ == "?") || (other.predicat_ == "?"))
            && ((object_ == other.object_) || (object_ == "?") || (other.object_ == "?")));
  }

  friend std::ostream& operator<<(std::ostream& os, const Fact& fact)
  {
    os << fact.toString();
    return os;
  }

  std::string subject_;
  std::string predicat_;
  std::string object_;
  bool add_;

protected:
  static std::regex regex;
  static std::smatch match;

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
