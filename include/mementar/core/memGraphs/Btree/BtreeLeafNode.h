#ifndef MEMENTAR_BTREELEAFNODE_H
#define MEMENTAR_BTREELEAFNODE_H

#include "mementar/core/memGraphs/Btree/BtreeNode.h"
#include "mementar/core/memGraphs/Btree/BtreeLeaf.h"

namespace mementar
{

template<typename Tkey, typename Tdata, typename Tnode>
class BtreeLeafNode : public BtreeNode<Tkey,Tdata,Tnode>
{
  static_assert(std::is_base_of<DllNode<Tdata>,Tnode>::value, "Tnode must be derived from DllNode");
public:
  BtreeLeafNode(size_t order = 10) : BtreeNode<Tkey,Tdata,Tnode>(order)
  {}

  ~BtreeLeafNode() {}

  BtreeLeaf<Tkey,Tdata,Tnode>* insert(const Tkey& key, const Tdata& data);
  bool remove(const Tkey& key, const Tdata& data);
  BtreeLeaf<Tkey,Tdata,Tnode>* find(const Tkey& key);
  BtreeLeaf<Tkey,Tdata,Tnode>* findNear(const Tkey& key);
  BtreeLeaf<Tkey,Tdata,Tnode>* getFirst();

  virtual void display(size_t depth = 0);
private:
  std::vector<BtreeLeaf<Tkey,Tdata,Tnode>*> leafs_;

  virtual bool needBalancing();
  virtual void split();
};

template<typename Tkey, typename Tdata, typename Tnode>
BtreeLeaf<Tkey,Tdata,Tnode>* BtreeLeafNode<Tkey,Tdata,Tnode>::insert(const Tkey& key, const Tdata& data)
{
  BtreeLeaf<Tkey,Tdata,Tnode>* res = nullptr;

  if(leafs_.size() == 0)
  {
    this->keys_.push_back(key);
    res = new BtreeLeaf<Tkey,Tdata,Tnode>(key, data);
    leafs_.push_back(res);
    res->setMother(this);
    return res;
  }
  else
  {
    BtreeLeaf<Tkey,Tdata,Tnode>* last = nullptr;
    if(leafs_.size())
      last = leafs_.back();

    if(key > this->keys_.back())
    {
      this->keys_.push_back(key);
      res = new BtreeLeaf<Tkey,Tdata,Tnode>(key, data);
      leafs_.push_back(res);
      res->setNextNode(last->getNextNode());
      last->setNextNode(res);
      res->setPreviousNode(last);
      res->setMother(this);
    }
    else if(this->keys_.back() == key)
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
            res = new BtreeLeaf<Tkey,Tdata,Tnode>(key, data);
            // here last is the next node of res
            last = leafs_[i];
            res->setNextNode(last);
            res->setPreviousNode(last->getPreviousNode());
            if(res->getPreviousNode())
              res->getPreviousNode()->setNextNode(res);
            if(res->getNextNode())
              res->getNextNode()->setPreviousNode(res);
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

template<typename Tkey, typename Tdata, typename Tnode>
bool BtreeLeafNode<Tkey,Tdata,Tnode>::remove(const Tkey& key, const Tdata& data)
{
  for(size_t i = 0; i < this->keys_.size(); i++)
  {
    if(this->keys_[i] == key)
    {
      leafs_[i]->remove(data);
      if(leafs_[i]->getData().size() == 0)
      {
        if(leafs_[i]->getPreviousNode() != nullptr)
          leafs_[i]->getPreviousNode()->setNextNode(leafs_[i]->getNextNode());
        if(leafs_[i]->getNextNode() != nullptr)
          leafs_[i]->getNextNode()->setPreviousNode(leafs_[i]->getPreviousNode());
        delete leafs_[i];
        leafs_.erase(leafs_.begin() + i);
        this->keys_.erase(this->keys_.begin() + i);

        if(leafs_.size() == 0)
          std::cout << "a node is empty but will not be destroyed" << std::endl;
      }
      return true;
    }
  }
  return false;
}

template<typename Tkey, typename Tdata, typename Tnode>
BtreeLeaf<Tkey,Tdata,Tnode>* BtreeLeafNode<Tkey,Tdata,Tnode>::find(const Tkey& key)
{
  for(size_t i = 0; i < this->keys_.size(); i++)
  {
    if(this->keys_[i] == key)
      return leafs_[i];
  }
  return nullptr;
}

template<typename Tkey, typename Tdata, typename Tnode>
BtreeLeaf<Tkey,Tdata,Tnode>* BtreeLeafNode<Tkey,Tdata,Tnode>::findNear(const Tkey& key)
{
  for(size_t i = 0; i < this->keys_.size(); i++)
  {
    if(this->keys_[i] >= key)
      return leafs_[i];
  }
  return static_cast<BtreeLeaf<Tkey,Tdata,Tnode>*>(leafs_[leafs_.size() - 1]->getNextNode());
}

template<typename Tkey, typename Tdata, typename Tnode>
BtreeLeaf<Tkey,Tdata,Tnode>* BtreeLeafNode<Tkey,Tdata,Tnode>::getFirst()
{
  return leafs_[0];
}

template<typename Tkey, typename Tdata, typename Tnode>
bool BtreeLeafNode<Tkey,Tdata,Tnode>::needBalancing()
{
  return (leafs_.size() > this->order_);
}

template<typename Tkey, typename Tdata, typename Tnode>
void BtreeLeafNode<Tkey,Tdata,Tnode>::split()
{
  BtreeLeafNode<Tkey,Tdata,Tnode>* new_node = new BtreeLeafNode<Tkey,Tdata,Tnode>(this->order_);

  size_t half_order = this->order_/2;
  for(size_t i = 0; i < half_order; i++)
  {
    new_node->leafs_.insert(new_node->leafs_.begin(), leafs_.back());
    leafs_.pop_back();
    new_node->leafs_[i]->setMother(new_node);

    new_node->keys_.insert(new_node->keys_.begin(), this->keys_.back());
    this->keys_.pop_back();
  }

  if(this->mother_ != nullptr)
  {
    this->mother_->insert(new_node, new_node->keys_[0]);
  }
  else
  {
    BtreeNode<Tkey,Tdata,Tnode>* new_mother = new BtreeNode<Tkey,Tdata,Tnode>(this->order_);
    new_mother->setLevel(this->level_ + 1);
    new_mother->insert(this, this->keys_[0]);
    new_mother->insert(new_node, new_node->keys_[0]);
  }
}

template<typename Tkey, typename Tdata, typename Tnode>
void BtreeLeafNode<Tkey,Tdata,Tnode>::display(size_t depth)
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
