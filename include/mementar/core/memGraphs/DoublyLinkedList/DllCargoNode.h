#ifndef MEMENTAR_DLLCARGONODE_H
#define MEMENTAR_DLLCARGONODE_H

#include "mementar/core/memGraphs/DoublyLinkedList/DllNode.h"

namespace mementar {

class DllLinkedElement;

class DllCargoNode : public DllNode<DllLinkedElement*>
{
public:
  DllCargoNode(DllLinkedElement* data);
  virtual ~DllCargoNode() {}

  virtual void push_back(DllLinkedElement* data);
  virtual void remove(DllLinkedElement* data);

protected:
  void unlinkDll(size_t i);
};

} // namespace mementar

#endif // MEMENTAR_DLLCARGONODE_H
