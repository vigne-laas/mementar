#ifndef MEMENTAR_LINKEDBTREE_H
#define MEMENTAR_LINKEDBTREE_H

#include "mementar/core/Btree/Btree.h"
#include "mementar/core/LinkedBtree/LinkedBtreeLeafNode.h"
#include "mementar/core/LinkedFact.h"

namespace mementar
{

template<typename Tkey>
class LinkedBtree : public Btree<Tkey, LinkedFact>
{
public:
  LinkedBtree(size_t order = 10) : Btree<Tkey, LinkedFact>(order) {}

  size_t insert(const Tkey& key, LinkedFact* data);

private:
};

template<typename Tkey>
size_t LinkedBtree<Tkey>::insert(const Tkey& key, LinkedFact* data)
{
  this->nb_data_++;
  if(this->last_ == nullptr)
  {
    this->root_ = new LinkedBtreeLeafNode<Tkey>(this->order_);
    this->level_ = this->root_->getLevel();
    this->last_ = this->root_->insert(key, data);
  }
  else
  {
    BtreeLeaf<Tkey, LinkedFact>* tmp = this->root_->insert(key, data);
    if(tmp != nullptr)
    {
      if(tmp->operator>(this->last_))
        this->last_ = tmp;
      if(this->root_->getMother() != nullptr)
      {
        this->root_ = this->root_->getMother();
        this->level_ = this->root_->getLevel();
      }
    }
  }

  return this->nb_data_;
}

} // namespace mementar

#endif // MEMENTAR_LINKEDBTREE_H
