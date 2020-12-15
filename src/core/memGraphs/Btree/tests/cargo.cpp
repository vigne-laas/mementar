#include <iostream>
#include <chrono>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include "mementar/core/memGraphs/ExtendedBtree/DlBtree.h"

using namespace std::chrono;

int main()
{
  mementar::DlBtree<int, int, 3> tree;

  for(size_t i = 0; i < 10; i++)
    tree.insert(i, i);

  std::cout << "*****" << std::endl;

  tree.insert(3, 10);

  tree.remove(3, 3);

  tree.displayTree();

  using LeafTyep = mementar::BplusLeaf<int, mementar::LinkedData<int, mementar::DlLeaf<int, int> > >;
  for(LeafTyep* leaf = tree.getFirst(); leaf != nullptr; leaf = leaf->getNextLeaf())
  {
    for(auto data : leaf->payload_)
    {
      std::cout << data.value_ << " -> ";
      if(data.getNextLeaf() != nullptr)
      {
        for(auto d : data.getNextLeaf()->getData())
          std::cout << d.value_ << " : ";
      }
      std::cout << std::endl;
    }
  }

  return 0;
}
