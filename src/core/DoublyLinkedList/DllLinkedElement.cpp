#include "mementar/core/DoublyLinkedList/DllLinkedElement.h"

namespace mementar {

DllLinkedElement::DllLinkedElement()
{
  dll_node_ = nullptr;
}

DllCargoNode* DllLinkedElement::getPreviousDllNode()
{
  if(dll_node_ != nullptr)
    return static_cast<DllCargoNode*>(dll_node_->getPreviousNode());
  else
   return nullptr;
}

DllCargoNode* DllLinkedElement::getNextDllNode()
{
  if(dll_node_ != nullptr)
    return static_cast<DllCargoNode*>(dll_node_->getNextNode());
  else
   return nullptr;
}

std::vector<DllLinkedElement*> DllLinkedElement::getPreviousDllData()
{
  if(dll_node_ != nullptr)
  {
    if(dll_node_->getPreviousNode() != nullptr)
      return dll_node_->getPreviousNode()->getData();
    else
      return {};
  }
  else
    return {};
}

std::vector<DllLinkedElement*> DllLinkedElement::getNextDllData()
{
  if(dll_node_ != nullptr)
  {
    if(dll_node_->getNextNode() != nullptr)
      return dll_node_->getNextNode()->getData();
    else
      return {};
  }
  else
    return {};
}

} // namespace mementar
