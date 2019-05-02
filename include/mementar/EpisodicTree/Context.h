#ifndef MEMENTAR_CONTEXT_H
#define MEMENTAR_CONTEXT_H

#include <map>
#include <string>
#include <ctime>

#include "mementar/Fact.h"

namespace mementar
{

class Context
{
public:
  void insert(const Fact& fact);
  void remove(const Fact& fact);

  bool exist(const std::string& name);
  bool subjectExist(const std::string& subject);
  bool predicatExist(const std::string& predicat);
  bool objectExist(const std::string& object);

  std::string toString();
  void fromString(const std::string& string);

  static void storeContexts(std::vector<Context>& contexts, std::vector<time_t>& keys, const std::string& directory);
private:
  std::map<std::string, size_t> subjects_;
  std::map<std::string, size_t> predicats_;
  std::map<std::string, size_t> objects_;
};

} // namespace mementar

#endif // MEMENTAR_CONTEXT_H
