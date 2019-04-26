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
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  mementar::CompressedLeafNode<size_t> compressed_node("/home/gsarthou/Desktop/tests");
  for(size_t i = 200000; i < 210000; i++)
  {
    //std::cout << i << std::endl;
    compressed_node.insert(i, mementar::Fact("bob", "hasValue", std::to_string(i)));
  }

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  std::cout << "took " << time_span.count() << std::endl;

  /*compressed_node.remove(102,mementar::Fact("bob", "hasValue", std::to_string(102)));
  compressed_node.insert(0, mementar::Fact("bob", "hasValue", std::to_string(0)));
  //compressed_node.display(150000);

  std::cout << "first key = " << compressed_node.getFirst()->getKey() << std::endl;

  std::cout << "find key = " << compressed_node.find(50)->getKey() << std::endl;
  std::cout << "findNear key = " << compressed_node.findNear(102)->getKey() << std::endl;
*/
  //mementar::CompressedLeaf<size_t> leaf(&tree, "/home/gsarthou/Desktop");

  return 0;
}
