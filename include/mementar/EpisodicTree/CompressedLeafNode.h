#ifndef MEMENTAR_COMPRESSEDLEAFNODE_H
#define MEMENTAR_COMPRESSEDLEAFNODE_H

#include <vector>

#include "mementar/Fact.h"
#include "mementar/Btree/Btree.h"
#include "mementar/EpisodicTree/CompressedLeaf.h"

namespace mementar
{

template<typename Tkey>
class CompressedLeafNode
{
public:
  CompressedLeafNode(std::string directory, size_t order = 10)
  {
    directory_ = directory;
    order_ = order;
    last_tree_nb_leafs_ = 0;
  }

  ~CompressedLeafNode()
  {
    for(auto tree : btree_childs_)
      delete tree;
  }

  void insert(const Tkey& key, const Fact& data);
  void remove(const Tkey& key, const Fact& data);
  BtreeLeaf<Tkey, Fact>* find(const Tkey& key);
  BtreeLeaf<Tkey, Fact>* findNear(const Tkey& key);
  BtreeLeaf<Tkey, Fact>* getFirst();

  void display(Tkey key);

private:
  std::string directory_;
  size_t order_;

  // keys_.size() == btree_childs_.size() + compressed_childs_.size()
  // keys_[i] correspond to the first key of child i
  std::vector<Tkey> keys_;
  std::vector<Btree<Tkey,Fact>*> btree_childs_;
  std::vector<CompressedLeaf<Tkey>> compressed_childs_;
  size_t last_tree_nb_leafs_;

  bool useNewTree();
  int getKeyIndex(const Tkey& key);
};

template<typename Tkey>
void CompressedLeafNode<Tkey>::insert(const Tkey& key, const Fact& data)
{
  if(keys_.size() == 0)
  {
    btree_childs_.push_back(new Btree<Tkey,Fact>(order_));
    keys_.push_back(key);
    last_tree_nb_leafs_ = btree_childs_[0]->insert(key, data);
  }
  else
  {
    if(key < keys_[0])
    {
      std::cout << "[ERROR] try to insert fact in past that do not exist" << std::endl;
      return;
    }

    size_t index = getKeyIndex(key);

    if(index < compressed_childs_.size())
    {
      if((index == compressed_childs_.size() - 1) && (keys_.size() == compressed_childs_.size()))
      {
        btree_childs_.push_back(new Btree<Tkey,Fact>(order_));
        keys_.push_back(key);
        last_tree_nb_leafs_ = btree_childs_[0]->insert(key, data);
      }
      else
        std::cout << "[ERROR] try to insrt fact in compressed file" << std::endl;
    }
    else if(useNewTree())
    {
      btree_childs_.push_back(new Btree<Tkey,Fact>(order_));
      keys_.push_back(key);
      last_tree_nb_leafs_ = btree_childs_[btree_childs_.size() - 1]->insert(key, data);
    }
    else if(index - keys_.size() + 1 == 0) // if insert in more recent tree
      last_tree_nb_leafs_ = btree_childs_[index - compressed_childs_.size()]->insert(key,data);
    else
      btree_childs_[index - compressed_childs_.size()]->insert(key,data);
  }
}

template<typename Tkey>
void CompressedLeafNode<Tkey>::remove(const Tkey& key, const Fact& data)
{
  int index = getKeyIndex(key);

  if(index >= 0)
  {
    if((size_t)index < compressed_childs_.size())
      std::cout << "[ERROR] try to remove fact from compressed file" << std::endl;
    else
      btree_childs_[index - compressed_childs_.size()]->remove(key,data);
  }
}

template<typename Tkey>
BtreeLeaf<Tkey, Fact>* CompressedLeafNode<Tkey>::find(const Tkey& key)
{
  int index = getKeyIndex(key);

  if(index >= 0)
  {
    if((size_t)index < compressed_childs_.size())
    {
      std::cout << "[TODO] find in compressed file" << std::endl;
      return nullptr;
    }
    else
      return btree_childs_[index - compressed_childs_.size()]->find(key);
  }
  else
    return nullptr;
}

template<typename Tkey>
BtreeLeaf<Tkey, Fact>* CompressedLeafNode<Tkey>::findNear(const Tkey& key)
{
  int index = getKeyIndex(key);

  if(index >= 0)
  {
    if((size_t)index < compressed_childs_.size())
    {
      std::cout << "[TODO] findNear in compressed file" << std::endl;
      return nullptr;
    }
    else
      return btree_childs_[index - compressed_childs_.size()]->findNear(key);
  }
  else
    return nullptr;
}

template<typename Tkey>
BtreeLeaf<Tkey, Fact>* CompressedLeafNode<Tkey>::getFirst()
{
  if(compressed_childs_.size())
  {
    std::cout << "[TODO] findFirst in compressed file" << std::endl;
    return nullptr;
  }
  else if(btree_childs_.size())
    return btree_childs_[0]->getFirst();
  else
    return nullptr;
}

template<typename Tkey>
void CompressedLeafNode<Tkey>::display(Tkey key)
{
  int index = getKeyIndex(key);

  if(index >= 0)
  {
    if((size_t)index < compressed_childs_.size())
      std::cout << compressed_childs_[index].getDirectoty() << std::endl;
    else
      return btree_childs_[index - compressed_childs_.size()]->display();
  }
}

template<typename Tkey>
bool CompressedLeafNode<Tkey>::useNewTree()
{
  if(last_tree_nb_leafs_ >= 100000)
    return true;
  else
    return false;
}

template<typename Tkey>
int CompressedLeafNode<Tkey>::getKeyIndex(const Tkey& key)
{
  int index = keys_.size() - 1;
  for(size_t i = 0; i < keys_.size(); i++)
  {
    if(key < keys_[i])
    {
      index = i - 1;
      break;
    }
  }
  return index;
}

} // mementar

#endif // MEMENTAR_COMPRESSEDLEAFNODE_H
