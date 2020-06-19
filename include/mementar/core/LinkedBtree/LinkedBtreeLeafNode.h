#ifndef MEMENTAR_LINKEDBTREELEAFNODE_H
#define MEMENTAR_LINKEDBTREELEAFNODE_H

#include "mementar/core/memGraphs/Btree/BtreeLeafNode.h"
#include "mementar/core/Data/LinkedFact.h"

#include "mementar/core/utility/Display.h"

namespace mementar
{

template<typename Tkey>
class LinkedBtreeLeafNode : public BtreeLeafNode<Tkey, LinkedFact<Tkey>>
{
  typedef LinkedFact<Tkey> LinkedFact_;
  typedef BtreeLeaf<Tkey,LinkedFact_> LinkedBtreeLeaf_;
public:
  LinkedBtreeLeafNode(size_t order = 10) : BtreeLeafNode<Tkey,LinkedFact_>(order)
  {}

  BtreeLeaf<Tkey,LinkedFact_>* insert(LinkedFact_* data);
  bool remove(const LinkedFact_& data);

private:
  void split();

  LinkedFact_* getPrev(LinkedBtreeLeaf_* current, LinkedFact_* data);
  std::vector<LinkedFact_*> getPrevs(LinkedBtreeLeaf_* current, LinkedFact_* data);
  LinkedFact_* getNext(LinkedBtreeLeaf_* current, LinkedFact_* data);
  std::vector<LinkedFact_*> getNexts(LinkedBtreeLeaf_* current, LinkedFact_* data);

  void linkPrev(LinkedFact_* current, LinkedFact_* prev, LinkedFact_* next);
  void linkNext(LinkedFact_* current, LinkedFact_* next, LinkedFact_* prev);

  void unlink(LinkedFact_* current_data, LinkedBtreeLeaf_* current_leaf);
  void unlinkPrev(LinkedFact_* current, std::vector<LinkedFact_*> prev, LinkedFact_* next);
  void unlinkNext(LinkedFact_* current, std::vector<LinkedFact_*> next, LinkedFact_* prev);
};

template<typename Tkey>
BtreeLeaf<Tkey,LinkedFact<Tkey>>* LinkedBtreeLeafNode<Tkey>::insert(LinkedFact<Tkey>* data)
{
  LinkedBtreeLeaf_* res = BtreeLeafNode<Tkey, LinkedFact<Tkey>>::insert(data);

  if(res != nullptr)
  {
    auto next = getNext(res, data);
    auto prev = getPrev(res, data);

    linkPrev(data, prev, next);
    linkNext(data, next, prev);
  }

  return res;
}

template<typename Tkey>
bool LinkedBtreeLeafNode<Tkey>::remove( const LinkedFact<Tkey>& data)
{
  for(size_t i = 0; i < this->keys_.size(); i++)
  {
    if(this->keys_[i] == data.getStamp())
    {
      LinkedFact_* current_data = this->leafs_[i]->getData(data);
      unlink(current_data, this->leafs_[i]);
      this->leafs_[i]->remove(data);
      if(this->leafs_[i]->getData().size() == 0)
      {
        if(this->leafs_[i]->prev_ != nullptr)
          this->leafs_[i]->prev_->next_ = this->leafs_[i]->next_;
        if(this->leafs_[i]->next_ != nullptr)
          this->leafs_[i]->next_->prev_ = this->leafs_[i]->prev_;
        delete this->leafs_[i];
        this->leafs_.erase(this->leafs_.begin() + i);
        this->keys_.erase(this->keys_.begin() + i);

        if(this->leafs_.size() == 0)
          Display::Info("a node is empty but will not be destroyed");
      }
      return true;
    }
  }
  return false;
}

template<typename Tkey>
void LinkedBtreeLeafNode<Tkey>::split()
{
  LinkedBtreeLeafNode<Tkey>* new_node = new LinkedBtreeLeafNode<Tkey>(this->order_);

  size_t half_order = this->order_/2;
  for(size_t i = 0; i < half_order; i++)
  {
    new_node->leafs_.insert(new_node->leafs_.begin(), this->leafs_[this->leafs_.size() - 1]);
    this->leafs_.erase(this->leafs_.begin() + this->leafs_.size() - 1);
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
    BtreeNode<Tkey,LinkedFact_>* new_mother = new BtreeNode<Tkey,LinkedFact_>(this->order_);
    new_mother->setLevel(this->level_ + 1);
    new_mother->insert(this, this->keys_[0]);
    new_mother->insert(new_node, new_node->keys_[0]);
  }
}

template<typename Tkey>
LinkedFact<Tkey>* LinkedBtreeLeafNode<Tkey>::getPrev(BtreeLeaf<Tkey,LinkedFact<Tkey>>* current, LinkedFact<Tkey>* data)
{
  LinkedFact_* res = nullptr;

  LinkedBtreeLeaf_* leaf = current->prev_;
  while(leaf != nullptr)
  {
    for(auto ld : leaf->getData())
    {
      if(data->isEventPart(*ld))
        return ld;
    }
    leaf = leaf->prev_;
  }

  return res;
}

template<typename Tkey>
std::vector<LinkedFact<Tkey>*> LinkedBtreeLeafNode<Tkey>::getPrevs(BtreeLeaf<Tkey,LinkedFact<Tkey>>* current, LinkedFact<Tkey>* data)
{
  std::vector<LinkedFact_*> res;
  bool end = false;

  LinkedBtreeLeaf_* leaf = current->prev_;
  if(leaf == nullptr)
    end = true;

  while(!end)
  {
    for(auto ld : leaf->getData())
    {
      if(data->isEventPart(*ld))
      {
        res.push_back(ld);
        if(ld->prev_->next_ == ld)
          end = true;
        break;
      }
    }

    leaf = leaf->prev_;
    if(leaf == nullptr)
      end = true;
  }

  return res;
}

template<typename Tkey>
LinkedFact<Tkey>* LinkedBtreeLeafNode<Tkey>::getNext(BtreeLeaf<Tkey,LinkedFact<Tkey>>* current, LinkedFact<Tkey>* data)
{
  LinkedFact_* res = nullptr;

  LinkedBtreeLeaf_* leaf = current->next_;
  while(leaf != nullptr)
  {
    for(auto ld : leaf->getData())
    {
      if(data->isEventPart(*ld))
        return ld;
    }
    leaf = leaf->next_;
  }

  return res;
}

template<typename Tkey>
std::vector<LinkedFact<Tkey>*> LinkedBtreeLeafNode<Tkey>::getNexts(BtreeLeaf<Tkey,LinkedFact<Tkey>>* current, LinkedFact<Tkey>* data)
{
  std::vector<LinkedFact_*> res;
  bool end = false;

  LinkedBtreeLeaf_* leaf = current->next_;
  if(leaf == nullptr)
    end = true;

  while(!end)
  {
    for(auto ld : leaf->getData())
    {
      if(data->isEventPart(*ld))
      {
        res.push_back(ld);
        if(ld->next_->prev_ == ld)
          end = true;
        break;
      }
    }

    leaf = leaf->next_;
    if(leaf == nullptr)
      end = true;
  }

  return res;
}

template<typename Tkey>
void LinkedBtreeLeafNode<Tkey>::linkPrev(LinkedFact<Tkey>* current, LinkedFact<Tkey>* prev, LinkedFact<Tkey>* next)
{
  if(prev == nullptr)
    return;

  if(current->operator==(*prev))
  {
    if(next == nullptr)
    {
      current->toLinkNext = prev->toLinkNext;
      prev->toLinkNext.clear();
      current->toLinkNext.push_back(prev);
    }
    current->prev_ = prev->prev_;
  }
  else
  {
    current->prev_ = prev;
    prev->next_ = current;
    if(prev->toLinkNext.size())
    {
      for(auto d : prev->toLinkNext)
        d->next_ = current;
      prev->toLinkNext.clear();
    }
  }
}

template<typename Tkey>
void LinkedBtreeLeafNode<Tkey>::linkNext(LinkedFact<Tkey>* current, LinkedFact<Tkey>* next, LinkedFact<Tkey>* prev)
{
  if(next == nullptr)
    return;

  if(current->operator==(*next))
  {
    if(prev == nullptr)
    {
      current->toLinkPrev = next->toLinkPrev;
      next->toLinkPrev.clear();
      current->toLinkPrev.push_back(next);
    }
    current->next_ = next->next_;
  }
  else
  {
    current->next_ = next;
    next->prev_ = current;
    if(next->toLinkPrev.size())
    {
      for(auto d : next->toLinkPrev)
        d->prev_ = current;
      next->toLinkPrev.clear();
    }
  }
}

template<typename Tkey>
void LinkedBtreeLeafNode<Tkey>::unlink(LinkedFact<Tkey>* current_data, BtreeLeaf<Tkey,LinkedFact<Tkey>>* current_leaf)
{
  std::vector<LinkedFact_*> prev;
  std::vector<LinkedFact_*> next;

  prev = getPrevs(current_leaf, current_data);
  next = getNexts(current_leaf, current_data);

  unlinkPrev(current_data, prev, next.size() ? next[0] : nullptr);
  unlinkNext(current_data, next, prev.size() ? prev[0] : nullptr);
}

template<typename Tkey>
void LinkedBtreeLeafNode<Tkey>::unlinkPrev(LinkedFact<Tkey>* current, std::vector<LinkedFact<Tkey>*> prev, LinkedFact<Tkey>* next)
{
  if(prev.size())
  {
    if(!current->operator==(*prev[0]))
    {
      if(!current->operator==(*next))
      {
        for(auto p : prev)
          p->next_ = current->next_;
      }
      else
      {
        for(auto p : prev)
          p->next_ = next;
      }
    }
  }
  else if(current->toLinkPrev.size())
  {
    if(current->operator==(*next))
    {
      next->toLinkPrev = std::move(current->toLinkPrev);
      next->toLinkPrev.pop_back();
    }
  }
}

template<typename Tkey>
void LinkedBtreeLeafNode<Tkey>::unlinkNext(LinkedFact<Tkey>* current, std::vector<LinkedFact<Tkey>*> next, LinkedFact<Tkey>* prev)
{
  if(next.size())
  {
    if(!current->operator==(*next[0]))
    {
      if(!current->operator==(*prev))
      {
        for(auto n : next)
          n->prev_ = current->prev_;
      }
      else
      {
        for(auto n : next)
          n->prev_ = prev;
      }
    }
  }
  else if(current->toLinkNext.size())
  {
    if(current->operator==(*prev))
    {
      prev->toLinkNext = std::move(current->toLinkNext);
      prev->toLinkNext.pop_back();
    }
  }
}

} // namespace mementar

#endif // MEMENTAR_LINKEDBTREELEAFNODE_H
