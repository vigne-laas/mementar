#ifndef MEMENTAR_BTREENODE_H
#define MEMENTAR_BTREENODE_H

#include <vector>

#include "mementar/core/memGraphs/Btree/BtreeLeaf.h"

namespace mementar
{

template<typename Tkey, typename Tdata, typename Tnode>
class BtreeNode
{
  static_assert(std::is_base_of<DllNode<Tdata>,Tnode>::value, "Tnode must be derived from DllNode");
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

  virtual BtreeLeaf<Tkey,Tdata,Tnode>* insert(const Tkey& key, const Tdata& data);
  void insert(BtreeNode<Tkey,Tdata,Tnode>* new_node, const Tkey& key);
  virtual bool remove(const Tkey& key, const Tdata& data);
  virtual BtreeLeaf<Tkey,Tdata,Tnode>* find(const Tkey& key);
  virtual BtreeLeaf<Tkey,Tdata,Tnode>* findNear(const Tkey& key);
  virtual BtreeLeaf<Tkey,Tdata,Tnode>* getFirst();

  void setMother(BtreeNode<Tkey,Tdata,Tnode>* mother) { mother_ = mother; }
  BtreeNode<Tkey,Tdata,Tnode>* getMother() { return mother_; }

  void setLevel(size_t level) { level_ = level; }
  size_t getLevel() { return level_; }

  size_t getNbChilds() { return childs_.size(); }

  virtual void display(size_t depth = 0);
protected:
  std::vector<Tkey> keys_;
  std::vector<BtreeNode<Tkey,Tdata,Tnode>*> childs_;
  BtreeNode<Tkey,Tdata,Tnode>* mother_;
  size_t order_;
  size_t level_;

  virtual bool needBalancing();
  virtual void split();
};

template<typename Tkey, typename Tdata, typename Tnode>
BtreeLeaf<Tkey,Tdata,Tnode>* BtreeNode<Tkey,Tdata,Tnode>::insert(const Tkey& key, const Tdata& data)
{
  size_t index;
  for(index = 0; index < this->keys_.size(); index++)
  {
    if(key < this->keys_[index])
      break;
  }
  return childs_[index]->insert(key, data);
}

template<typename Tkey, typename Tdata, typename Tnode>
void BtreeNode<Tkey,Tdata,Tnode>::insert(BtreeNode<Tkey,Tdata,Tnode>* new_node, const Tkey& key)
{
  if(childs_.size() == 0)
  {
    childs_.push_back(new_node);
    new_node->setMother(this);
    return;
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
          this->childs_.insert(this->childs_.begin() + i + 1, new_node);
          new_node->setMother(this);
          break;
        }
      }
    }
  }

  if(needBalancing())
    split();
}

template<typename Tkey, typename Tdata, typename Tnode>
bool BtreeNode<Tkey,Tdata,Tnode>::remove(const Tkey& key, const Tdata& data)
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

template<typename Tkey, typename Tdata, typename Tnode>
BtreeLeaf<Tkey,Tdata,Tnode>* BtreeNode<Tkey,Tdata,Tnode>::find(const Tkey& key)
{
  for(size_t i = 0; i < this->keys_.size(); i++)
  {
    if(this->keys_[i] > key)
      return this->childs_[i]->find(key);
  }
  return this->childs_[this->keys_.size()]->find(key);
}

template<typename Tkey, typename Tdata, typename Tnode>
BtreeLeaf<Tkey,Tdata,Tnode>* BtreeNode<Tkey,Tdata,Tnode>::findNear(const Tkey& key)
{
  for(size_t i = 0; i < this->keys_.size(); i++)
  {
    if(this->keys_[i] > key)
      return this->childs_[i]->findNear(key);
  }
  return this->childs_[this->keys_.size()]->findNear(key);
}

template<typename Tkey, typename Tdata, typename Tnode>
BtreeLeaf<Tkey,Tdata,Tnode>* BtreeNode<Tkey,Tdata,Tnode>::getFirst()
{
  return this->childs_[0]->getFirst();
}

template<typename Tkey, typename Tdata, typename Tnode>
bool BtreeNode<Tkey,Tdata,Tnode>::needBalancing()
{
  return (childs_.size() > order_);
}

template<typename Tkey, typename Tdata, typename Tnode>
void BtreeNode<Tkey,Tdata,Tnode>::split()
{
  BtreeNode<Tkey,Tdata,Tnode>* new_node = new BtreeNode<Tkey,Tdata,Tnode>(order_);

  size_t half_order = order_/2 - 1;
  for(size_t i = 0; i < half_order; i++)
  {
    new_node->childs_.insert(new_node->childs_.begin(), childs_.back());
    childs_.pop_back();
    new_node->childs_[i]->setMother(new_node);

    new_node->keys_.insert(new_node->keys_.begin(), keys_.back());
    keys_.pop_back();
  }

  new_node->childs_.insert(new_node->childs_.begin(), childs_.back());
  childs_.pop_back();
  new_node->childs_[half_order]->setMother(new_node);

  if(mother_ != nullptr)
  {
    mother_->insert(new_node, keys_.back());
    keys_.pop_back();
  }
  else
  {
    BtreeNode<Tkey,Tdata,Tnode>* new_mother = new BtreeNode<Tkey,Tdata,Tnode>(order_);
    new_mother->setLevel(this->level_ + 1);
    new_mother->insert(this, keys_.back());
    new_mother->insert(new_node, keys_.back());
    keys_.pop_back();
  }
}

template<typename Tkey, typename Tdata, typename Tnode>
void BtreeNode<Tkey,Tdata,Tnode>::display(size_t depth)
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
