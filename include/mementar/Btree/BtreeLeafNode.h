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

  BtreeLeaf<Tkey,Tdata>* insert(const Tkey& key, const Tdata& data);
  void remove(const Tkey& key, const Tdata& data);
  BtreeLeaf<Tkey, Tdata>* find(const Tkey& key);
  BtreeLeaf<Tkey, Tdata>* findNear(const Tkey& key);
  BtreeLeaf<Tkey, Tdata>* getFirst();

  virtual void display(size_t depth = 0);
private:
  std::vector<BtreeLeaf<Tkey,Tdata>*> leafs_;

  virtual bool needBalancing();
  void split();
};

template<typename Tkey, typename Tdata>
BtreeLeaf<Tkey,Tdata>* BtreeLeafNode<Tkey,Tdata>::insert(const Tkey& key, const Tdata& data)
{
  BtreeLeaf<Tkey,Tdata>* res = nullptr;

  if(leafs_.size() == 0)
  {
    this->keys_.push_back(key);
    res = new BtreeLeaf<Tkey,Tdata>(key, data);
    leafs_.push_back(res);
    res->setMother(this);
  }
  else
  {
    BtreeLeaf<Tkey,Tdata>* last = nullptr;
    if(leafs_.size())
      last = leafs_[leafs_.size() - 1];

    if(key > this->keys_[this->keys_.size() - 1])
    {
      this->keys_.push_back(key);
      res = new BtreeLeaf<Tkey,Tdata>(key, data);
      leafs_.push_back(res);
      last->next_ = res;
      res->prev_ = last;
      res->setMother(this);
    }
    else if(this->keys_[this->keys_.size() - 1] == key)
    {
      last->push_back(data);
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
            if(res->prev_)
              res->prev_->next_ = res;
            if(res->next_)
              res->next_->prev_ = res;
            this->keys_.insert(this->keys_.begin() + i, key);
            this->leafs_.insert(this->leafs_.begin() + i, res);
            res->setMother(this);
          }
          break;
        }
      }
    }
  }

  if(needBalancing())
    split();

  return res;
}

template<typename Tkey, typename Tdata>
void BtreeLeafNode<Tkey,Tdata>::remove(const Tkey& key, const Tdata& data)
{
  for(size_t i = 0; i < this->keys_.size(); i++)
  {
    if(this->keys_[i] == key)
    {
      leafs_[i]->remove(data);
      if(leafs_[i]->getData().size() == 0)
      {
        if(leafs_[i]->prev_ != nullptr)
          leafs_[i]->prev_->next_ = leafs_[i]->next_;
        if(leafs_[i]->next_ != nullptr)
          leafs_[i]->next_->prev_ = leafs_[i]->prev_;
        delete leafs_[i];
        leafs_.erase(leafs_.begin() + i);
        this->keys_.erase(this->keys_.begin() + i);

        if(leafs_.size() == 0)
          std::cout << "a node is empty but will not be destroyed" << std::endl;
      }
      return;
    }
  }
}

template<typename Tkey, typename Tdata>
BtreeLeaf<Tkey, Tdata>* BtreeLeafNode<Tkey,Tdata>::find(const Tkey& key)
{
  for(size_t i = 0; i < this->keys_.size(); i++)
  {
    if(this->keys_[i] == key)
      return leafs_[i];
  }
  return nullptr;
}

template<typename Tkey, typename Tdata>
BtreeLeaf<Tkey, Tdata>* BtreeLeafNode<Tkey,Tdata>::findNear(const Tkey& key)
{
  for(size_t i = 0; i < this->keys_.size(); i++)
  {
    if(this->keys_[i] >= key)
      return leafs_[i];
  }
  return leafs_[leafs_.size() - 1]->next_;
}

template<typename Tkey, typename Tdata>
BtreeLeaf<Tkey, Tdata>* BtreeLeafNode<Tkey,Tdata>::getFirst()
{
  return leafs_[0];
}

template<typename Tkey, typename Tdata>
bool BtreeLeafNode<Tkey,Tdata>::needBalancing()
{
  return (leafs_.size() > this->order_);
}

template<typename Tkey, typename Tdata>
void BtreeLeafNode<Tkey,Tdata>::split()
{
  BtreeLeafNode<Tkey,Tdata>* new_node = new BtreeLeafNode<Tkey,Tdata>(this->order_);

  size_t half_order = this->order_/2;
  for(size_t i = 0; i < half_order; i++)
  {
    new_node->leafs_.insert(new_node->leafs_.begin(), leafs_[leafs_.size() - 1]);
    leafs_.erase(leafs_.begin() + leafs_.size() - 1);
    new_node->leafs_[i]->setMother(new_node);

    new_node->keys_.insert(new_node->keys_.begin(), this->keys_[this->keys_.size() - 1]);
    this->keys_.erase(this->keys_.begin() + this->keys_.size() - 1);
  }

  if(this->mother_ != nullptr)
  {
    this->mother_->insert(new_node, new_node->keys_[0]);
  }
  else
  {
    BtreeNode<Tkey,Tdata>* new_mother = new BtreeNode<Tkey,Tdata>(this->order_);
    new_mother->setLevel(this->level_ + 1);
    new_mother->insert(this, new_node->keys_[0]);
    new_mother->insert(new_node, new_node->keys_[0]);
  }
}

template<typename Tkey, typename Tdata>
void BtreeLeafNode<Tkey,Tdata>::display(size_t depth)
{
  for(size_t i = 0; i < this->keys_.size(); i++)
  {
    for(size_t j = 0; j < depth; j++)
      std::cout << "\t";
    std::vector<Tdata> datas = leafs_[i]->getData();
    std::cout << this->keys_[i] << " => ";
    for(auto data : datas)
      std::cout << data << " : ";
    std::cout << std::endl;
  }
}

} // namespace mementar

#endif // MEMENTAR_BTREELEAFNODE_H
