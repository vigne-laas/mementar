#include <iostream>
#include <chrono>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include "mementar/EpisodicTree/CompressedLeaf.h"
#include "mementar/Btree/Btree.h"

using namespace std::chrono;

int main()
{
  mementar::Btree<size_t, mementar::Fact> tree(10);
  for(size_t i = 0; i < 1000; i++)
    tree.insert(i, mementar::Fact("bob", "hasValue", std::to_string(i)));

  tree.display();

  mementar::CompressedLeaf<size_t> leaf(&tree, "/home/gsarthou/Desktop");

  return 0;
}
