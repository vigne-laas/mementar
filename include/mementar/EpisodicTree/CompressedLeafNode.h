#ifndef MEMENTAR_COMPRESSEDLEAFNODE_H
#define MEMENTAR_COMPRESSEDLEAFNODE_H

#include <vector>
#include <thread>
#include <atomic>
#include <shared_mutex>

#include "mementar/Fact.h"
#include "mementar/Btree/Btree.h"
#include "mementar/EpisodicTree/CompressedLeaf.h"
#include "mementar/EpisodicTree/Context.h"

namespace mementar
{

class ArchivedLeaf;

class CompressedLeafNode
{
  friend ArchivedLeaf;
public:
  CompressedLeafNode(std::string directory, size_t order = 10);
  ~CompressedLeafNode();

  CompressedLeafNode* split();

  void insert(const time_t& key, const Fact& data);
  void remove(const time_t& key, const Fact& data);
  BtreeLeaf<time_t, Fact>* find(const time_t& key);
  BtreeLeaf<time_t, Fact>* findNear(const time_t& key);
  BtreeLeaf<time_t, Fact>* getFirst();
  BtreeLeaf<time_t, Fact>* getLast();

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
  size_t order_;
  mutable std::shared_timed_mutex mut_;

  // keys_.size() == btree_childs_.size() + compressed_childs_.size()
  // keys_[i] correspond to the first key of child i
  std::vector<time_t> keys_;
  std::vector<Context> contexts_;
  std::vector<Btree<time_t,Fact>*> btree_childs_;
  std::vector<CompressedLeaf> compressed_childs_;
  std::vector<Btree<time_t,Fact>*> compressed_sessions_tree_;
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

  void loadStoredData();
  void insert(const time_t& key, const CompressedLeaf& leaf);

  void compressFirst();
  void createSession(size_t index);

  void clean();
};

} // mementar

#endif // MEMENTAR_COMPRESSEDLEAFNODE_H
