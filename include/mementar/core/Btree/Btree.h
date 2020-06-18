#ifndef MEMENTAR_BTREE_H
#define MEMENTAR_BTREE_H

#include <iostream>
#include <math.h>

#include "mementar/core/Btree/BtreeNode.h"
#include "mementar/core/Btree/BtreeLeafNode.h"
#include "mementar/core/Btree/BtreeLeaf.h"

#include "mementar/core/DoublyLinkedList/DllNode.h"

namespace mementar
{

template<typename Tkey, typename Tdata, typename Tnode = DllNode<Tdata>>
class Btree
{
  static_assert(std::is_base_of<DllNode<Tdata>,Tnode>::value, "Tnode must be derived from DllNode");
public:
  Btree(size_t order = 10)
  {
    root_ = nullptr;
    last_ = nullptr;
    order_ = order;
    level_ = 0;
    nb_data_ = 0;
  }

  ~Btree()
  {
    if(root_ != nullptr)
      delete root_;

    BtreeLeaf<Tkey, Tdata, Tnode>* tmp;
    while(last_ != nullptr)
    {
      tmp = last_;
      last_ = static_cast<BtreeLeaf<Tkey, Tdata, Tnode>*>(last_->getPreviousNode());
      delete tmp;
    }
  }

  size_t insert(const Tkey& key, const Tdata& data);
  bool remove(const Tkey& key, const Tdata& data);
  BtreeLeaf<Tkey, Tdata, Tnode>* find(const Tkey& key);
  BtreeLeaf<Tkey, Tdata, Tnode>* findNear(const Tkey& key);
  BtreeLeaf<Tkey, Tdata, Tnode>* getFirst();
  BtreeLeaf<Tkey, Tdata, Tnode>* getLast() { return last_; }

  size_t estimateMinLeaves()
  {
    return pow((double)order_/2. + 1., (double)level_ - 1.) * root_->getNbChilds();
  }

  size_t estimateMaxLevel(size_t nbLeafs)
  {
    return log((double)nbLeafs/2.) / log((double)order_/2. + 1) + 1;
  }

  size_t getCurrentLevel() { return level_; }

  void display(int count = -1);

private:
  BtreeNode<Tkey,Tdata,Tnode>* root_;
  BtreeLeaf<Tkey, Tdata, Tnode>* last_;
  size_t order_;
  size_t level_;
  size_t nb_data_;
};

template<typename Tkey, typename Tdata, typename Tnode>
size_t Btree<Tkey,Tdata,Tnode>::insert(const Tkey& key, const Tdata& data)
{
  nb_data_++;
  if(last_ == nullptr)
  {
    root_ = new BtreeLeafNode<Tkey,Tdata,Tnode>(order_);
    level_ = root_->getLevel();
    last_ = root_->insert(key, data);
  }
  else
  {
    BtreeLeaf<Tkey,Tdata,Tnode>* tmp = root_->insert(key, data);
    if(tmp != nullptr)
    {
      if(tmp->operator>(last_))
        last_ = tmp;
      if(root_->getMother() != nullptr)
      {
        root_ = root_->getMother();
        level_ = root_->getLevel();
      }
    }
  }

  return nb_data_;
}

template<typename Tkey, typename Tdata, typename Tnode>
bool Btree<Tkey,Tdata,Tnode>::remove(const Tkey& key, const Tdata& data)
{
  nb_data_--;
  if(root_ != nullptr)
    return root_->remove(key, data);
  return false;
}

template<typename Tkey, typename Tdata, typename Tnode>
BtreeLeaf<Tkey,Tdata,Tnode>* Btree<Tkey,Tdata,Tnode>::find(const Tkey& key)
{
  if(root_ != nullptr)
    return root_->find(key);
  else
    return nullptr;
}

template<typename Tkey, typename Tdata, typename Tnode>
BtreeLeaf<Tkey,Tdata,Tnode>* Btree<Tkey,Tdata,Tnode>::findNear(const Tkey& key)
{
  if(root_ != nullptr)
    return root_->findNear(key);
  else
    return nullptr;
}

template<typename Tkey, typename Tdata, typename Tnode>
BtreeLeaf<Tkey,Tdata,Tnode>* Btree<Tkey,Tdata,Tnode>::getFirst()
{
  if(root_ != nullptr)
    return root_->getFirst();
  else
    return nullptr;
}

template<typename Tkey, typename Tdata, typename Tnode>
void Btree<Tkey,Tdata,Tnode>::display(int count)
{
  BtreeLeaf<Tkey,Tdata,Tnode>* tmp = last_;
  int cpt = 0;
  std::cout << "******" << std::endl;
  while((tmp != nullptr) && ((cpt < count) || (count == -1)))
  {
    std::vector<Tdata> datas = tmp->getData();
    std::cout << tmp->getKey() << " => ";
    for(const auto& data : datas)
      std::cout << data << " : ";
    std::cout << std::endl;
    tmp = static_cast<BtreeLeaf<Tkey,Tdata,Tnode>*>(tmp->getPreviousNode());
    cpt++;
  }
  std::cout << "******" << std::endl;
  root_->display();
}

} // namespace mementar

#endif // MEMENTAR_BTREE_H
