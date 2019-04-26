#include <iostream>
#include <chrono>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include "mementar/EpisodicTree/CompressedLeaf.h"
#include "mementar/Btree/Btree.h"
#include "mementar/EpisodicTree/CompressedLeafNode.h"

using namespace std::chrono;

int main()
{
  mementar::CompressedLeafNode<size_t> compressed_node;
  for(size_t i = 1; i < 120; i++)
  {
    compressed_node.insert(i, mementar::Fact("bob", "hasValue", std::to_string(i)));
  }

  compressed_node.remove(102,mementar::Fact("bob", "hasValue", std::to_string(102)));
  compressed_node.insert(0, mementar::Fact("bob", "hasValue", std::to_string(0)));
  compressed_node.display(1);

  std::cout << "first key = " << compressed_node.getFirst()->getKey() << std::endl;

  std::cout << "find key = " << compressed_node.find(50)->getKey() << std::endl;
  std::cout << "findNear key = " << compressed_node.findNear(102)->getKey() << std::endl;

  //mementar::CompressedLeaf<size_t> leaf(&tree, "/home/gsarthou/Desktop");

  return 0;
}
