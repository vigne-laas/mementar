#ifndef MEMENTAR_BTREELEAFNODE_H
#define MEMENTAR_BTREELEAFNODE_H

#include "mementar/Btree/BtreeNode.h"
#include "mementar/Btree/BtreeLeaf.h"

namespace mementar
{

template<typename Tkey, typename Tdata>
class BtreeLeafNode : public BtreeNode<Tkey,Tdata>
{
public:
  BtreeLeafNode(size_t order = 10) : BtreeNode<Tkey,Tdata>(order)
  {}

  ~BtreeLeafNode() {}

  virtual bool isLeafNode() { return true; }

  BtreeLeaf<Tkey,Tdata>* insert(const Tkey& key, const Tdata& data);
  virtual bool needBalancing();
private:
  std::vector<BtreeLeaf<Tkey,Tdata>*> leafs_;
};

template<typename Tkey, typename Tdata>
BtreeLeaf<Tkey,Tdata>* BtreeLeafNode<Tkey,Tdata>::insert(const Tkey& key, const Tdata& data)
{
  BtreeLeaf<Tkey,Tdata>* res = nullptr;
  BtreeLeaf<Tkey,Tdata>* last = nullptr;
  if(leafs_.size())
    last = leafs_[leafs_.size() - 1];

  if(leafs_.size() == 0)
  {
    this->keys_.push_back(key);
    res = new BtreeLeaf<Tkey,Tdata>(key, data);
    leafs_.push_back(res);
    res->setMother(this);
  }
  else
  {
    if(this->keys_[this->keys_.size() - 1] == key)
    {
      last->push_back(data);
    }
    else if(key > this->keys_[this->keys_.size() - 1])
    {
      this->keys_.push_back(key);
      res = new BtreeLeaf<Tkey,Tdata>(key, data);
      leafs_.push_back(res);
      last->next_ = res;
      res->prev_ = last;
      res->setMother(this);
    }
    else
    {
      for(size_t i = 0; i < this->keys_.size(); i++)
      {
        if(this->keys_[i] >= key)
        {
          if(this->keys_[i] == key)
          {
            leafs_[i]->push_back(data);
          }
          else
          {
            res = new BtreeLeaf<Tkey,Tdata>(key, data);
            // here last is the next node of res
            last = leafs_[i];
            res->next_ = last;
            res->prev_ = last->prev_;
            res->prev_->next_ = res;
            res->next_->prev_ = res;
            res->setMother(this);
          }
          break;
        }
      }
    }
  }

  return res;
}

template<typename Tkey, typename Tdata>
bool BtreeLeafNode<Tkey,Tdata>::needBalancing()
{
  return (leafs_.size() > this->order_);
}

} // namespace mementar

#endif // MEMENTAR_BTREELEAFNODE_H
