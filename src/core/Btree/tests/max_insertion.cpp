#include <iostream>
#include <chrono>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include "mementar/core/Btree/BtreeLeaf.h"
#include "mementar/core/Btree/Btree.h"

using namespace std::chrono;

int main()
{
  size_t nb = 349526;

  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  mementar::Btree<size_t, size_t> tree(5);
  for(size_t i = 0; true; i++)
    if(tree.insert(i, i) >= nb)
      break;

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  std::cout << "took " << time_span.count() << std::endl;

  //tree.display();
  std::cout << "level = " << tree.estimateMaxLevel(nb) << ">=" << tree.getCurrentLevel() << std::endl;

  return 0;
}
