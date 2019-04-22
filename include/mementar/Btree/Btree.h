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
  Btree()
  {
    root_ = nullptr;
    last_ = nullptr;
  }

  ~Btree()
  {
    if(root_ != nullptr)
      delete root_;

    if(last_ != nullptr)
      delete last_;
  }

  void insert(const Tkey& key, const Tdata& data);

private:
  BtreeNode<Tkey, Tdata>* root_;
  BtreeLeaf<Tkey, Tdata>* last_;
};

template<typename Tkey, typename Tdata>
void Btree<Tkey,Tdata>::insert(const Tkey& key, const Tdata& data)
{
  if(last_ == nullptr)
  {
    root_ = new BtreeLeafNode<Tkey, Tdata>();
    last_ = root_->insert(key, data);
  }
  else
  {
    BtreeLeaf<Tkey, Tdata>* tmp = root_->insert(key, data);
    if(tmp != nullptr)
    {
      if(tmp->operator>(last_))
        last_ = tmp;
      if(tmp->getMother()->needBalancing())
      {
        std::cout << "needBalancing" << std::endl;
      }
    }

  }
}

} // namespace mementar

#endif // MEMENTAR_BTREE_H
