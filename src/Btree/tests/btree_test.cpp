#include <iostream>
#include <chrono>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include "mementar/Btree/BtreeLeaf.h"
#include "mementar/Btree/Btree.h"

using namespace std::chrono;

int main()
{
  mementar::BtreeLeaf<int, int> leaf1(0, 2);
  mementar::BtreeLeaf<int, int> leaf2(1, 3);

  if(leaf1.operator<(&leaf2))
    std::cout << "inf" << std::endl;
  else
    std::cout << "sup" << std::endl;

  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  size_t nb = 300000;
  mementar::Btree<size_t, size_t> tree(10);
  for(size_t i = 0; i < nb; i=i+2)
  {
    tree.insert(i, i);
  }

  for(size_t i = 1; i < nb; i=i+2)
  {
    tree.insert(i, i);
  }
  //tree.display();

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  std::cout << "took " << time_span.count() << std::endl;

  return 0;
}
