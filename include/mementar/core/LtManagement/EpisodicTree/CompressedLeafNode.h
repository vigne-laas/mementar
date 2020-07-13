#ifndef MEMENTAR_COMPRESSEDLEAFNODE_H
#define MEMENTAR_COMPRESSEDLEAFNODE_H

#include <vector>
#include <thread>
#include <atomic>
#include <shared_mutex>

#include "mementar/core/memGraphs/Branchs/types/Event.h"
#include "mementar/core/memGraphs/Btree/BplusTree.h"
#include "mementar/core/LtManagement/EpisodicTree/CompressedLeaf.h"
#include "mementar/core/LtManagement/EpisodicTree/Context.h"

namespace mementar
{

class ArchivedLeaf;

class CompressedLeafNode
{
  friend ArchivedLeaf;
  using LeafType = typename BplusLeaf<time_t, Event*>::LeafType;
public:
  CompressedLeafNode(const std::string& directory);
  ~CompressedLeafNode();

  CompressedLeafNode* split();

  void insert(Event* data);
  void remove(Event* data);
  LeafType* find(const time_t& key);
  LeafType* findNear(const time_t& key);
  LeafType* getFirst();
  LeafType* getLast();

  void display(time_t key);
  size_t size() { return keys_.size(); }

  std::string getDirectory() { return directory_; }
  time_t getKey()
  {
    if(contexts_.size())
      return contexts_[0].getKey();
    else
      return -1;
  }

  void newSession() { ask_for_new_tree_ = true; }

private:
  CompressedLeafNode() {};
  void init();

  std::string directory_;
  mutable std::shared_timed_mutex mut_;

  // keys_.size() == btree_childs_.size() + compressed_childs_.size()
  // keys_[i] correspond to the first key of child i
  std::vector<time_t> keys_;
  std::vector<Context> contexts_;
  std::vector<BplusTree<time_t, Event*>*> btree_childs_;
  std::vector<CompressedLeaf> compressed_childs_;
  std::vector<BplusTree<time_t, Event*>*> compressed_sessions_tree_;
  std::vector<int> compressed_sessions_timeout_; //ms
  std::vector<bool> modified_;

  size_t last_tree_nb_leafs_;
  time_t earlier_key_;
  bool ask_for_new_tree_;

  std::atomic<bool> running_;
  std::thread session_cleaner_;

  inline void createNewTreeChild(const time_t& key);
  inline bool useNewTree();
  inline int getKeyIndex(const time_t& key);

  bool loadStoredData();
  void insert(const time_t& key, const CompressedLeaf& leaf);

  void compressFirst();
  void createSession(size_t index);

  void clean();
};

} // mementar

#endif // MEMENTAR_COMPRESSEDLEAFNODE_H
