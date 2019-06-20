#ifndef MEMENTAR_ARCHIVEDLEAFNODE_H
#define MEMENTAR_ARCHIVEDLEAFNODE_H

#include "mementar/EpisodicTree/ArchivedLeaf.h"
#include "mementar/EpisodicTree/CompressedLeafNode.h"
#include "mementar/EpisodicTree/CompressedLeafNodeSession.h"

namespace mementar
{

class ArchivedLeafNode
{
public:
  ArchivedLeafNode(const std::string& directory, size_t order = 10);
  ~ArchivedLeafNode();

  void insert(const time_t& key, const Fact& data);
  void remove(const time_t& key, const Fact& data);
  BtreeLeaf<time_t, Fact>* find(const time_t& key);
  BtreeLeaf<time_t, Fact>* findNear(const time_t& key);
  BtreeLeaf<time_t, Fact>* getFirst();
  BtreeLeaf<time_t, Fact>* getLast();

  void display(time_t key);

  void newSession();

private:
  std::string directory_;
  size_t order_;
  mutable std::shared_timed_mutex mut_;

  // keys_.size() == btree_childs_.size() + compressed_childs_.size()
  // keys_[i] correspond to the first key of child i
  std::vector<time_t> keys_;
  std::vector<CompressedLeafNode*> compressed_childs_;
  std::vector<ArchivedLeaf> archived_childs_;
  std::vector<CompressedLeafNodeSession*> archived_sessions_tree_;
  std::vector<int> archived_sessions_timeout_; //ms
  std::vector<bool> modified_;

  time_t earlier_key_;
  bool ask_for_new_tree_;

  std::atomic<bool> running_;
  std::thread session_cleaner_;

  void createNewCompressedChild(const time_t& key);
  bool useNewTree();
  int getKeyIndex(const time_t& key);

  void loadStoredData();
  void insert(const time_t& key, const ArchivedLeaf& leaf);
  void archiveFirst();

  void createSession(size_t index);

  void clean();
};

} // mementar

#endif // MEMENTAR_ARCHIVEDLEAFNODE_H
