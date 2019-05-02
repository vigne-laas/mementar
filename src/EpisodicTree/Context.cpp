#include "mementar/EpisodicTree/Context.h"

#include <regex>

namespace mementar
{

void Context::insert(const Fact& fact)
{
  std::map<std::string, size_t>::iterator it;

  it = subjects_.find(fact.subject_);
  if(it != subjects_.end())
    it->second++;
  else
    subjects_[fact.subject_] = 1;

  it = predicats_.find(fact.predicat_);
  if(it != predicats_.end())
    it->second++;
  else
    predicats_[fact.predicat_] = 1;

  if(fact.object_.find(":") == std::string::npos)
  {
    it = objects_.find(fact.object_);
    if(it != objects_.end())
      it->second++;
    else
      objects_[fact.object_] = 1;
  }
}

void Context::remove(const Fact& fact)
{
  std::map<std::string, size_t>::iterator it;

  it = subjects_.find(fact.subject_);
  if(it != subjects_.end())
    it->second--;

  it = predicats_.find(fact.predicat_);
  if(it != predicats_.end())
    it->second--;

  it = objects_.find(fact.object_);
  if(it != objects_.end())
    it->second--;
}

bool Context::exist(const std::string& name)
{
  if(subjects_.find(name) != subjects_.end())
    return true;
  else if(predicats_.find(name) != predicats_.end())
    return true;
  else if(objects_.find(name) != objects_.end())
    return true;
  else
    return true;
}

bool Context::subjectExist(const std::string& subject)
{
  if(subjects_.find(subject) != subjects_.end())
    return true;
  return false;
}

bool Context::predicatExist(const std::string& predicat)
{
  if(predicats_.find(predicat) != predicats_.end())
    return true;
  return false;
}

bool Context::objectExist(const std::string& object)
{
  if(objects_.find(object) != objects_.end())
    return true;
  return false;
}

std::string Context::toString()
{
  std::string res;
  res += "__SUBJECT__\n";
  for(auto it : subjects_)
    res += "<" + std::to_string(it.second) + ">" + it.first + "\n";

  res += "__PREDICAT__\n";
  for(auto it : predicats_)
    res += "<" + std::to_string(it.second) + ">" + it.first + "\n";

  res += "__OBJECT__\n";
  for(auto it : objects_)
    res += "<" + std::to_string(it.second) + ">" + it.first + "\n";

  return res;
}

void Context::fromString(const std::string& string)
{
  std::regex section("__(\\w+)__");
  std::regex context("<(\\d+)>(\\w+)");
  std::smatch match;

  std::map<std::string, size_t>* map_ptr = nullptr;

  std::istringstream iss(string);
  std::string line;
  while(std::getline(iss, line))
  {
    if(std::regex_match(line, match, section))
    {
      if(match[1].str() == "SUBJECT")
        map_ptr = &subjects_;
      else if(match[1].str() == "PREDICAT")
        map_ptr = &predicats_;
      else if(match[1].str() == "OBJECT")
        map_ptr = &objects_;
      else
        map_ptr = nullptr;
    }
    else if(std::regex_match(line, match, context))
    {
      if(map_ptr != nullptr)
      {
        size_t nb;
        std::istringstream iss(match[1].str());
        iss >> nb;
        map_ptr->operator[](match[2].str()) = nb;
      }
    }
  }
}

} // mementar
