#include <iostream>
#include <chrono>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <ctime>
#include <unistd.h>

#include "mementar/core/EpisodicTree/CompressedLeaf.h"
#include "mementar/core/Btree/Btree.h"
#include "mementar/core/EpisodicTree/CompressedLeafNode.h"

using namespace std::chrono;

int main()
{
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  mementar::CompressedLeafNode compressed_node("/home/gsarthou/Desktop/tests");

  std::cout << " *************" << std::endl;
  for(size_t i = 0; i < 400000; i++)
  {
    //std::cout << i << std::endl;
    compressed_node.insert(i, mementar::Fact("bob", "hasValue", std::to_string(i)));
    //usleep(1);
  }
  std::cout << " *************" << std::endl;

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  std::cout << "took " << time_span.count() << std::endl;

  compressed_node.remove(102,mementar::Fact("bob", "hasValue", std::to_string(102)));
  compressed_node.insert(0, mementar::Fact("bob", "hasValue", std::to_string(0)));

  std::cout << " *************" << std::endl;
  /*//compressed_node.display(150000);

  std::cout << "first key = " << compressed_node.getFirst()->getKey() << std::endl;

  std::cout << "find key = " << compressed_node.find(50)->getKey() << std::endl;
  std::cout << "findNear key = " << compressed_node.findNear(102)->getKey() << std::endl;
*/
  //mementar::CompressedLeaf<size_t> leaf(&tree, "/home/gsarthou/Desktop");

  return 0;
}
