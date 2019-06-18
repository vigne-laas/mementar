#ifndef MEMENTAR_ARCHIVEDLEAF_H
#define MEMENTAR_ARCHIVEDLEAF_H

#include <ctime>
#include <string>

#include "mementar/EpisodicTree/CompressedLeafNode.h"

namespace mementar
{

class ArchivedLeaf
{
public:
  ArchivedLeaf(CompressedLeafNode* tree, size_t nb, const std::string& directory);
  ArchivedLeaf(const time_t& key, const std::string& directory);

  std::string getDirectoty() { return directory_ + ".mar"; }
  time_t getKey() { return key_; }

  Btree<time_t, Fact>* getTree(size_t i);
  std::vector<Context> getContexts();

private:
  time_t key_;
  std::string directory_;
};

} // namespace mementar

#endif // MEMENTAR_ARCHIVEDLEAF_H
