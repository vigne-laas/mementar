#include "mementar/core/memGraphs/EventLinkedList/EllNode.h"

#include "mementar/core/memGraphs/EventLinkedList/EllElement.h"

namespace mementar {

EllNode::EllNode() : DllCargoNode()
{
}

void EllNode::push_back(EllElement* data)
{
  DllCargoNode::push_back(data);
  link(data);
}

void EllNode::remove(EllElement* data)
{
  for(size_t i = 0; i < payload_.size();)
  {
    if(payload_[i]->operator==(data))
    {
      unlinkDll(i);

      unlink(dynamic_cast<EllElement*>(payload_[i]));

      payload_.erase(payload_.begin() + i);
    }
    else
      i++;
  }
}

void EllNode::link(EllElement* data)
{
  auto next = getNext(data);
  auto prev = getPrev(data);

  linkPrev(data, prev, next);
  linkNext(data, next, prev);
}

EllElement* EllNode::getPrev(EllElement* data)
{
  EllElement* res = nullptr;

  DllCargoNode* prev_node = dynamic_cast<DllCargoNode*>(data->getPreviousDllNode());
  while(prev_node != nullptr)
  {
    for(auto ld : prev_node->getData())
    {
      if(data->isEventPart(ld))
        return dynamic_cast<EllElement*>(ld);
    }
    prev_node = dynamic_cast<DllCargoNode*>(prev_node->getPreviousNode());
  }

  return res;
}

std::vector<EllElement*> EllNode::getPrevs(EllElement* data)
{
  std::vector<EllElement*> res;
  bool end = false;

  DllCargoNode* prev_node = dynamic_cast<DllCargoNode*>(data->getPreviousDllNode());
  if(prev_node == nullptr)
    end = true;

  while(!end)
  {
    for(auto ld : prev_node->getData())
    {
      if(data->isEventPart(ld))
      {
        res.push_back(dynamic_cast<EllElement*>(ld));
        if(dynamic_cast<EllElement*>(ld)->prev_elem_->next_elem_ == ld)
          end = true;
        break;
      }
    }

    prev_node = dynamic_cast<DllCargoNode*>(prev_node->getPreviousNode());
    if(prev_node == nullptr)
      end = true;
  }

  return res;
}

EllElement* EllNode::getNext(EllElement* data)
{
  EllElement* res = nullptr;

  DllCargoNode* next_node = dynamic_cast<DllCargoNode*>(data->getNextDllNode());
  while(next_node != nullptr)
  {
    for(auto ld : next_node->getData())
    {
      if(data->isEventPart(ld))
        return dynamic_cast<EllElement*>(ld);
    }
    next_node = dynamic_cast<DllCargoNode*>(next_node->getNextNode());
  }

  return res;
}

std::vector<EllElement*> EllNode::getNexts(EllElement* data)
{
  std::vector<EllElement*> res;
  bool end = false;

  DllCargoNode* next_node = dynamic_cast<DllCargoNode*>(data->getNextDllNode());
  if(next_node == nullptr)
    end = true;

  while(!end)
  {
    for(auto ld : next_node->getData())
    {
      if(data->isEventPart(ld))
      {
        res.push_back(dynamic_cast<EllElement*>(ld));
        if(dynamic_cast<EllElement*>(ld)->next_elem_->prev_elem_ == ld)
          end = true;
        break;
      }
    }

    next_node = dynamic_cast<DllCargoNode*>(next_node->getNextNode());
    if(next_node == nullptr)
      end = true;
  }

  return res;
}

void EllNode::linkPrev(EllElement* current, EllElement* prev, EllElement* next)
{
  if(prev == nullptr)
    return;

  if(current->operator==(prev))
  {
    if(next == nullptr)
    {
      current->to_link_next = prev->to_link_next;
      prev->to_link_next.clear();
      current->to_link_next.push_back(prev);
    }
    current->prev_elem_ = prev->prev_elem_;
  }
  else
  {
    current->prev_elem_ = prev;
    prev->next_elem_ = current;
    if(prev->to_link_next.size())
    {
      for(auto d : prev->to_link_next)
        d->next_elem_ = current;
      prev->to_link_next.clear();
    }
  }
}

void EllNode::linkNext(EllElement* current, EllElement* next, EllElement* prev)
{
  if(next == nullptr)
    return;

  if(current->operator==(next))
  {
    if(prev == nullptr)
    {
      current->to_link_prev = next->to_link_prev;
      next->to_link_prev.clear();
      current->to_link_prev.push_back(next);
    }
    current->next_elem_ = next->next_elem_;
  }
  else
  {
    current->next_elem_ = next;
    next->prev_elem_ = current;
    if(next->to_link_prev.size())
    {
      for(auto d : next->to_link_prev)
        d->prev_elem_ = current;
      next->to_link_prev.clear();
    }
  }
}

void EllNode::unlink(EllElement* current_data)
{
  std::vector<EllElement*> prev = getPrevs(current_data);
  std::vector<EllElement*> next = getNexts(current_data);

  unlinkPrev(current_data, prev, next.size() ? next[0] : nullptr);
  unlinkNext(current_data, next, prev.size() ? prev[0] : nullptr);
}

void EllNode::unlinkPrev(EllElement* current, std::vector<EllElement*> prev, EllElement* next)
{
  if(prev.size())
  {
    if(!current->operator==(prev[0]))
    {
      if(!current->operator==(next))
      {
        for(auto p : prev)
          p->next_elem_ = current->next_elem_;
      }
      else
      {
        for(auto p : prev)
          p->next_elem_ = next;
      }
    }
  }
  else if(current->to_link_prev.size())
  {
    if(current->operator==(next))
    {
      next->to_link_prev = std::move(current->to_link_prev);
      next->to_link_prev.pop_back();
    }
  }
}

void EllNode::unlinkNext(EllElement* current, std::vector<EllElement*> next, EllElement* prev)
{
  if(next.size())
  {
    if(!current->operator==(next[0]))
    {
      if(!current->operator==(prev))
      {
        for(auto n : next)
          n->prev_elem_ = current->prev_elem_;
      }
      else
      {
        for(auto n : next)
          n->prev_elem_ = prev;
      }
    }
  }
  else if(current->to_link_next.size())
  {
    if(current->operator==(prev))
    {
      prev->to_link_next = std::move(current->to_link_next);
      prev->to_link_next.pop_back();
    }
  }
}

} // namespace mementar
