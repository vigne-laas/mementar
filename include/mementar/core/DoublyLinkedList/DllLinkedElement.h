#ifndef MEMENTAR_DLLLINKEDELEMENT_H
#define MEMENTAR_DLLLINKEDELEMENT_H

#include "mementar/core/DoublyLinkedList/DllCargoNode.h"

#include <ostream>

namespace mementar {

class DllLinkedElement
{
  friend DllCargoNode;
public:
  DllLinkedElement();

  DllCargoNode* getPreviousDllNode();
  DllCargoNode* getNextDllNode();

  std::vector<DllLinkedElement*> getPreviousDllData();
  std::vector<DllLinkedElement*> getNextDllData();

  virtual bool operator==(const DllLinkedElement*) = 0;
  virtual void print(std::ostream& os) const { os << this; }
  friend std::ostream& operator<<(std::ostream& os, DllLinkedElement* elem) { elem->print(os); return os;}

private:
  DllCargoNode* dll_node_;
};

} // namespace mementar

#endif // MEMENTAR_DLLLINKEDELEMENT_H
