#ifndef MEMENTAR_COMPRESSEDLEAFNODESESSION_H
#define MEMENTAR_COMPRESSEDLEAFNODESESSION_H

#include <vector>
#include <thread>
#include <atomic>
#include <shared_mutex>

#include "mementar/core/LinkedFact.h"
#include "mementar/core/Btree/Btree.h"
#include "mementar/core/EpisodicTree/CompressedLeafSession.h"
#include "mementar/core/EpisodicTree/Context.h"

#include "mementar/core/archiving_compressing/archiving/Archive.h"
#include "mementar/core/archiving_compressing/archiving/Header.h"

namespace mementar
{

class CompressedLeafNodeSession
{
public:
  CompressedLeafNodeSession(const std::string& file_name);
  ~CompressedLeafNodeSession();

  void insert(const time_t& key, const LinkedFact* data);
  bool remove(const time_t& key, const LinkedFact& data);
  BtreeLeaf<time_t, LinkedFact>* find(const time_t& key);
  BtreeLeaf<time_t, LinkedFact>* findNear(const time_t& key);
  BtreeLeaf<time_t, LinkedFact>* getFirst();
  BtreeLeaf<time_t, LinkedFact>* getLast();

  time_t getKey()
  {
    if(contexts_.size())
      return contexts_[0].getKey();
    else
      return -1;
  }

private:
  std::string file_name_;
  mutable std::shared_timed_mutex mut_;

  Archive arch_;
  Header header_;

  // keys_.size() == btree_childs_.size() + compressed_childs_.size()
  // keys_[i] correspond to the first key of child i
  std::vector<Context> contexts_;
  std::vector<CompressedLeafSession> childs_;
  std::vector<Btree<time_t,LinkedFact>*> sessions_tree_;
  std::vector<bool> modified_;

  time_t earlier_key_;

  std::atomic<bool> running_;

  inline int getKeyIndex(const time_t& key);

  void loadData();

  void createSession(size_t index);

  std::vector<char> treeToRaw(size_t index);
};

}

#endif // MEMENTAR_COMPRESSEDLEAFNODESESSION_H
