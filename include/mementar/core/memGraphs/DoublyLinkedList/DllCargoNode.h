#ifndef MEMENTAR_DLLCARGONODE_H
#define MEMENTAR_DLLCARGONODE_H

#include "mementar/core/memGraphs/DoublyLinkedList/DllNode.h"

namespace mementar {

class DllLinkedElement;

class DllCargoNode : public DllNode<DllLinkedElement*>
{
public:
  DllCargoNode(DllLinkedElement* data);
  ~DllCargoNode() {}

  void push_back(DllLinkedElement* data);
  void remove(DllLinkedElement* data);
private:
};

} // namespace mementar

#endif // MEMENTAR_DLLCARGONODE_H
