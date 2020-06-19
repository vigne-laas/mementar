#include "mementar/core/memGraphs/DoublyLinkedList/DllCargoNode.h"

#include "mementar/core/memGraphs/DoublyLinkedList/DllLinkedElement.h"

#include <iostream>

namespace mementar {

DllCargoNode::DllCargoNode(DllLinkedElement* data) : DllNode(data)
{
  for(auto p : payload_)
    p->dll_node_ = this;
}

void DllCargoNode::push_back(DllLinkedElement* data)
{
  data->dll_node_ = this;
  DllNode::push_back(data);
}

void DllCargoNode::remove(DllLinkedElement* data)
{
  for(size_t i = 0; i < payload_.size();)
  {
    if(payload_[i]->operator==(data))
    {
      payload_[i]->dll_node_ = nullptr;
      payload_.erase(payload_.begin() + i);
    }
    else
      i++;
  }
}

} // namespace mementar
