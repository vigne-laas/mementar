#include <iostream>
#include <chrono>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <ctime>
#include <unistd.h>

#include "mementar/Btree/Btree.h"
#include "mementar/EpisodicTree/ArchivedLeafNode.h"

using namespace std::chrono;

int main()
{
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  mementar::ArchivedLeafNode archived_node("/home/gsarthou/Desktop/tests");

  std::cout << " *************" << std::endl;
  for(size_t i = 0; i < 400000; i++)
  {
    //std::cout << i << std::endl;
    archived_node.insert(i, mementar::Fact("bob", "hasValue", std::to_string(i)));
    //usleep(1);
  }
  std::cout << " *************" << std::endl;

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  std::cout << "took " << time_span.count() << std::endl;

  archived_node.remove(102,mementar::Fact("bob", "hasValue", std::to_string(102)));
  std::cout << "removed" << std::endl;
  archived_node.insert(0, mementar::Fact("bob", "hasValue", std::to_string(0)));
  std::cout << "inserted" << std::endl;

  std::cout << " *************" << std::endl;
  /*//archived_node.display(150000);

  std::cout << "first key = " << archived_node.getFirst()->getKey() << std::endl;

  std::cout << "find key = " << archived_node.find(50)->getKey() << std::endl;
  std::cout << "findNear key = " << archived_node.findNear(102)->getKey() << std::endl;
*/
  //mementar::CompressedLeaf<size_t> leaf(&tree, "/home/gsarthou/Desktop");

  return 0;
}
