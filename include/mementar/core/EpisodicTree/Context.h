#ifndef MEMENTAR_CONTEXT_H
#define MEMENTAR_CONTEXT_H

#include <map>
#include <string>
#include <ctime>

#include "mementar/core/memGraphs/Fact.h"

namespace mementar
{

class Context
{
public:
  Context(time_t key) { key_ = key; }

  void insert(const Fact& fact);
  void remove(const Fact& fact);

  bool exist(const std::string& name);
  bool subjectExist(const std::string& subject);
  bool predicatExist(const std::string& predicat);
  bool objectExist(const std::string& object);

  std::string toString();
  void fromString(const std::string& string);

  time_t getKey() { return key_; }
  void setKey(time_t key) {key_ = key; }

  static void storeContexts(std::vector<Context>& contexts, const std::string& directory);
  static std::string ContextsToString(std::vector<Context>& contexts);
  static void loadContexts(std::vector<Context>& contexts, const std::string& directory);
  static std::vector<Context> StringToContext(const std::string& str);
private:
  time_t key_;
  std::map<std::string, size_t> subjects_;
  std::map<std::string, size_t> predicats_;
  std::map<std::string, size_t> objects_;
};

} // namespace mementar

#endif // MEMENTAR_CONTEXT_H
