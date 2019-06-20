#ifndef MEMENTAR_BTREENODE_H
#define MEMENTAR_BTREENODE_H

#include <vector>

#include "mementar/core/Btree/BtreeLeaf.h"

namespace mementar
{

template<typename Tkey, typename Tdata>
class BtreeNode
{
public:
  BtreeNode(size_t order = 10)
  {
    order_ = order;
    level_ = 1;
    mother_ = nullptr;
  }

  virtual ~BtreeNode()
  {
    for(auto child : childs_)
      delete child;
  }

  virtual BtreeLeaf<Tkey,Tdata>* insert(const Tkey& key, const Tdata& data);
  void insert(BtreeNode<Tkey,Tdata>* new_node, const Tkey& key);
  virtual bool remove(const Tkey& key, const Tdata& data);
  virtual BtreeLeaf<Tkey, Tdata>* find(const Tkey& key);
  virtual BtreeLeaf<Tkey, Tdata>* findNear(const Tkey& key);
  virtual BtreeLeaf<Tkey, Tdata>* getFirst();

  void setMother(BtreeNode<Tkey,Tdata>* mother) { mother_ = mother; }
  BtreeNode<Tkey,Tdata>* getMother() { return mother_; }

  void setLevel(size_t level) { level_ = level; }
  size_t getLevel() { return level_; }

  size_t getNbChilds() { return childs_.size(); }

  virtual void display(size_t depth = 0);
protected:
  std::vector<Tkey> keys_;
  std::vector<BtreeNode<Tkey,Tdata>*> childs_;
  BtreeNode<Tkey,Tdata>* mother_;
  size_t order_;
  size_t level_;

  virtual bool needBalancing();
  virtual void split();
};

template<typename Tkey, typename Tdata>
BtreeLeaf<Tkey,Tdata>* BtreeNode<Tkey,Tdata>::insert(const Tkey& key, const Tdata& data)
{
  size_t index = childs_.size() - 1;
  for(size_t i = 0; i < this->keys_.size(); i++)
  {
    if(key < this->keys_[i])
    {
      index = i;
      break;
    }
  }
  return childs_[index]->insert(key, data);
}

template<typename Tkey, typename Tdata>
void BtreeNode<Tkey,Tdata>::insert(BtreeNode<Tkey,Tdata>* new_node, const Tkey& key)
{
  if(childs_.size() == 0)
  {
    childs_.push_back(new_node);
    new_node->setMother(this);
  }
  else
  {
    if(this->keys_.size() == 0)
    {
      this->keys_.push_back(key);
      childs_.push_back(new_node);
      new_node->setMother(this);
    }
    else if(key > this->keys_[this->keys_.size() - 1])
    {
      this->keys_.push_back(key);
      childs_.push_back(new_node);
      new_node->setMother(this);
    }
    else
    {
      for(size_t i = 0; i < this->keys_.size(); i++)
      {
        if(key < this->keys_[i])
        {
          this->keys_.insert(this->keys_.begin() + i, key);
          this->childs_.insert(this->childs_.begin() + i, new_node);
          new_node->setMother(this);
          break;
        }
      }
    }
  }

  if(needBalancing())
    split();
}

template<typename Tkey, typename Tdata>
bool BtreeNode<Tkey,Tdata>::remove(const Tkey& key, const Tdata& data)
{
  size_t index = this->keys_.size();
  for(size_t i = 0; i < this->keys_.size(); i++)
  {
    if(this->keys_[i] > key)
    {
      index = i;
      break;
    }
  }
  return this->childs_[index]->remove(key, data);

  if(this->keys_.size() == 0)
    std::cout << "a node is empty but will not be destroyed" << std::endl;
}

template<typename Tkey, typename Tdata>
BtreeLeaf<Tkey, Tdata>* BtreeNode<Tkey,Tdata>::find(const Tkey& key)
{
  for(size_t i = 0; i < this->keys_.size(); i++)
  {
    if(this->keys_[i] > key)
      return this->childs_[i]->find(key);
  }
  return this->childs_[this->keys_.size()]->find(key);
}

template<typename Tkey, typename Tdata>
BtreeLeaf<Tkey, Tdata>* BtreeNode<Tkey,Tdata>::findNear(const Tkey& key)
{
  for(size_t i = 0; i < this->keys_.size(); i++)
  {
    if(this->keys_[i] > key)
      return this->childs_[i]->findNear(key);
  }

  return this->childs_[this->keys_.size()]->findNear(key);
}

template<typename Tkey, typename Tdata>
BtreeLeaf<Tkey, Tdata>* BtreeNode<Tkey,Tdata>::getFirst()
{
  return this->childs_[0]->getFirst();
}

template<typename Tkey, typename Tdata>
bool BtreeNode<Tkey,Tdata>::needBalancing()
{
  return (childs_.size() > order_);
}

template<typename Tkey, typename Tdata>
void BtreeNode<Tkey,Tdata>::split()
{
  BtreeNode<Tkey,Tdata>* new_node = new BtreeNode<Tkey,Tdata>(order_);

  size_t half_order = order_/2;
  for(size_t i = 0; i < half_order; i++)
  {
    new_node->childs_.insert(new_node->childs_.begin(), childs_[childs_.size() - 1]);
    childs_.erase(childs_.begin() + childs_.size() - 1);
    new_node->childs_[i]->setMother(new_node);
  }

  half_order = half_order - 1;
  for(size_t i = 0; i < half_order; i++)
  {
    new_node->keys_.insert(new_node->keys_.begin(), keys_[keys_.size() - 1]);
    keys_.erase(keys_.begin() + keys_.size() - 1);
  }

  if(mother_ != nullptr)
  {
    mother_->insert(new_node, keys_[keys_.size() - 1]);
    keys_.erase(keys_.begin() + keys_.size() - 1);
  }
  else
  {
    BtreeNode<Tkey,Tdata>* new_mother = new BtreeNode<Tkey,Tdata>(order_);
    new_mother->setLevel(this->level_ + 1);
    new_mother->insert(this, keys_[keys_.size() - 1]);
    new_mother->insert(new_node, keys_[keys_.size() - 1]);
    keys_.erase(keys_.begin() + keys_.size() - 1);
  }
}

template<typename Tkey, typename Tdata>
void BtreeNode<Tkey,Tdata>::display(size_t depth)
{
  size_t depth_1 = depth + 1;

  for(size_t i = 0; i < keys_.size(); i++)
  {
    childs_[i]->display(depth_1);
    for(size_t j = 0; j < depth; j++)
      std::cout << "\t";
    std::cout << keys_[i] << std::endl;
  }
  childs_[keys_.size()]->display(depth_1);
}

} // namespace mementar

#endif // MEMENTAR_BTREENODE_H
