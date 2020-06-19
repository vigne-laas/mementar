#ifndef MEMENTAR_ELLNODE_H
#define MEMENTAR_ELLNODE_H

#include "mementar/core/memGraphs/DoublyLinkedList/DllCargoNode.h"

#include <vector>

namespace mementar {

class EllElement;

class EllNode : public DllCargoNode
{
public:
  EllNode(EllElement* data);
  ~EllNode() {}

  void push_back(EllElement* data);
  void remove(EllElement* data);
private:

  void link(EllElement* data);

  EllElement* getPrev(EllElement* data);
  std::vector<EllElement*> getPrevs(EllElement* data);

  EllElement* getNext(EllElement* data);
  std::vector<EllElement*> getNexts(EllElement* data);

  void linkPrev(EllElement* current, EllElement* prev, EllElement* next);
  void linkNext(EllElement* current, EllElement* next, EllElement* prev);

  void unlink(EllElement* current_data);

  void unlinkPrev(EllElement* current, std::vector<EllElement*> prev, EllElement* next);
  void unlinkNext(EllElement* current, std::vector<EllElement*> next, EllElement* prev);
};

} // namespace mementar

#endif // MEMENTAR_ELLNODE_H
