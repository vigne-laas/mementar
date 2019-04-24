#ifndef MEMENTAR_BTREE_H
#define MEMENTAR_BTREE_H

#include <iostream>

#include "mementar/Btree/BtreeNode.h"
#include "mementar/Btree/BtreeLeafNode.h"
#include "mementar/Btree/BtreeLeaf.h"

namespace mementar
{

template<typename Tkey, typename Tdata>
class Btree
{
public:
  Btree(size_t order)
  {
    root_ = nullptr;
    last_ = nullptr;
    order_ = order;
  }

  ~Btree()
  {
    if(root_ != nullptr)
      delete root_;

    BtreeLeaf<Tkey, Tdata>* tmp;
    while(last_ != nullptr)
    {
      tmp = last_;
      last_ = last_->prev_;
      delete tmp;
    }
  }

  void insert(const Tkey& key, const Tdata& data);
  void remove(const Tkey& key, const Tdata& data);
  BtreeLeaf<Tkey, Tdata>* find(const Tkey& key);
  BtreeLeaf<Tkey, Tdata>* findNear(const Tkey& key);
  BtreeLeaf<Tkey, Tdata>* getFirst();

  void display(int count = -1);

private:
  BtreeNode<Tkey, Tdata>* root_;
  BtreeLeaf<Tkey, Tdata>* last_;
  size_t order_;
};

template<typename Tkey, typename Tdata>
void Btree<Tkey,Tdata>::insert(const Tkey& key, const Tdata& data)
{
  if(last_ == nullptr)
  {
    root_ = new BtreeLeafNode<Tkey, Tdata>(order_);
    last_ = root_->insert(key, data);
  }
  else
  {
    BtreeLeaf<Tkey, Tdata>* tmp = root_->insert(key, data);
    if(tmp != nullptr)
    {
      if(tmp->operator>(last_))
        last_ = tmp;
      if(root_->getMother() != nullptr)
        root_ = root_->getMother();
    }
  }
}

template<typename Tkey, typename Tdata>
void Btree<Tkey,Tdata>::remove(const Tkey& key, const Tdata& data)
{
  if(root_ != nullptr)
    return root_->remove(key, data);
}

template<typename Tkey, typename Tdata>
BtreeLeaf<Tkey, Tdata>* Btree<Tkey,Tdata>::find(const Tkey& key)
{
  if(root_ != nullptr)
    return root_->find(key);
  else
    return nullptr;
}

template<typename Tkey, typename Tdata>
BtreeLeaf<Tkey, Tdata>* Btree<Tkey,Tdata>::findNear(const Tkey& key)
{
  if(root_ != nullptr)
    return root_->findNear(key);
  else
    return nullptr;
}

template<typename Tkey, typename Tdata>
BtreeLeaf<Tkey, Tdata>* Btree<Tkey,Tdata>::getFirst()
{
  if(root_ != nullptr)
    return root_->getFirst();
  else
    return nullptr;
}

template<typename Tkey, typename Tdata>
void Btree<Tkey,Tdata>::display(int count)
{
  BtreeLeaf<Tkey, Tdata>* tmp = last_;
  int cpt = 0;
  std::cout << "******" << std::endl;
  while((tmp != nullptr) && ((cpt < count) || (count == -1)))
  {
    std::vector<Tdata> datas = tmp->getData();
    std::cout << tmp->getKey() << " => ";
    for(const auto& data : datas)
      std::cout << data << " : ";
    std::cout << std::endl;
    tmp = tmp->prev_;
    cpt++;
  }
  std::cout << "******" << std::endl;
  root_->display();
}

} // namespace mementar

#endif // MEMENTAR_BTREE_H
