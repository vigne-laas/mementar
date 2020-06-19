#ifndef MEMENTAR_ELLELEMENT_H
#define MEMENTAR_ELLELEMENT_H

#include "mementar/core/memGraphs/DoublyLinkedList/DllLinkedElement.h"
#include "mementar/core/memGraphs/EventLinkedList/EllNode.h"

namespace mementar {

class EllElement : public DllLinkedElement
{
  friend EllNode;
public:
  EllElement();
  ~EllElement() {}

  EllElement* getPreviousEllElement() { return prev_elem_; }
  EllElement* getNextEllElement() { return next_elem_; }

  virtual bool operator==(const DllLinkedElement*) = 0;
  virtual bool isEventPart(const DllLinkedElement* other) = 0;
  virtual void print(std::ostream& os) const { os << this; }

private:
  EllElement* prev_elem_;
  EllElement* next_elem_;

  std::vector<EllElement*> to_link_prev;
  std::vector<EllElement*> to_link_next;
};

} // namespace mementar

#endif // MEMENTAR_ELLELEMENT_H
