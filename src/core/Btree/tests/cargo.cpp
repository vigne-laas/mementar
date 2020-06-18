#include <iostream>
#include <chrono>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include "mementar/core/Btree/Btree.h"
#include "mementar/core/DoublyLinkedList/DllLinkedElement.h"
#include "mementar/core/DoublyLinkedList/DllCargoNode.h"

using namespace std::chrono;
using namespace mementar;

class DllElem : public DllLinkedElement
{
public:
  DllElem(int data) { elem_data = data; }
  int elem_data;

  virtual void print(std::ostream& os) const
  {
    os << elem_data;
  }

private:
  bool operator==(const DllLinkedElement* other)
  {
    return elem_data == static_cast<const DllElem*>(other)->elem_data;
  }
};

void displayNext(DllElem* elem)
{
  std::cout << "-" << elem->elem_data << std::endl;
  for(auto d : elem->getNextDllData())
    displayNext(static_cast<DllElem*>(d));
}

int main()
{
  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  std::vector<DllElem*> container;

  mementar::Btree<size_t, DllLinkedElement*, DllCargoNode> tree(3);
  for(size_t i = 0; i < 10; i++)
  {
    container.push_back(new DllElem(i));
    tree.insert(i, container.back());
  }

  std::cout << "*****" << std::endl;

  container.push_back(new DllElem(10));
  tree.insert(3, container.back());

  tree.remove(3, container[3]);

  tree.display();

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  std::cout << "took " << time_span.count() << std::endl;

  for(auto elem : container)
  {
    std::cout << elem->elem_data << " -> ";
    for(auto d : elem->getNextDllData())
      std::cout << static_cast<DllElem*>(d)->elem_data << " : ";
    std::cout << std::endl;
  }

  displayNext(container[0]);

  return 0;
}
