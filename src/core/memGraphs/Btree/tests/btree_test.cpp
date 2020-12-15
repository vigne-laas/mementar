#include <iostream>
#include <chrono>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include "mementar/core/memGraphs/Btree/BplusTree.h"

using namespace std::chrono;

int main()
{
  mementar::BtreeLeafBase<int, mementar::BplusLeaf<int, int>> leaf1(0);
  mementar::BtreeLeafBase<int, mementar::BplusLeaf<int, int>> leaf2(1);

  if(leaf1.operator<(leaf2))
    std::cout << "inf" << std::endl;
  else
    std::cout << "sup" << std::endl;

  std::vector<size_t> sizes = {10,100,1000,10000,100000,1000000,10000000};
  std::vector<double> insert;
  std::vector<double> removed;
  std::vector<double> find;
  for(auto nb : sizes)
  //size_t nb = 0;
  {
    high_resolution_clock::time_point t1 = high_resolution_clock::now();

    mementar::BplusTree<int, int, 3> tree;
    for(size_t i = 0; i < nb; i++)
      tree.insert(i, i);

    high_resolution_clock::time_point t2 = high_resolution_clock::now();

    for(size_t i = 0; i < nb; i++)
      tree.find(i);

    high_resolution_clock::time_point t3 = high_resolution_clock::now();

    for(size_t i = 0; i < nb; i+=2)
      tree.remove(i, i);

    high_resolution_clock::time_point t4 = high_resolution_clock::now();

    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    insert.push_back(time_span.count());
    time_span = duration_cast<duration<double>>(t3 - t2);
    find.push_back(time_span.count());
    time_span = duration_cast<duration<double>>(t4 - t3);
    removed.push_back(time_span.count());

    for(size_t i = 0; i < sizes.size(); i++)
      std::cout << sizes[i] << ";";
    std::cout << std::endl;
    for(size_t i = 0; i < insert.size(); i++)
      std::cout << insert[i] << ";";
    std::cout << std::endl;
    for(size_t i = 0; i < find.size(); i++)
      std::cout << find[i] << ";";
    std::cout << std::endl;
    for(size_t i = 0; i < removed.size(); i++)
      std::cout << removed[i] << ";";
    std::cout << std::endl;
  }

  {
    mementar::BplusTree<int, int> tree;
    for(size_t i = 0; i < 50; i++)
      tree.insert(i, i);

    //tree.displayTree();

    auto res = tree.find(10);
    if(res) std::cout << res->getKey() << std::endl; else std::cout << "-" << std::endl;
    res = tree.find(28);
    if(res) std::cout << res->getKey() << std::endl; else std::cout << "-" << std::endl;
    res = tree.find(74);
    if(res) std::cout << res->getKey() << std::endl; else std::cout << "-" << std::endl;
    res = tree.find(53);
    if(res) std::cout << res->getKey() << std::endl; else std::cout << "-" << std::endl;
    res = tree.findNear(59);
    if(res) std::cout << res->getKey() << std::endl; else std::cout << "-" << std::endl;
    res = tree.getFirst();
    if(res) std::cout << res->getKey() << std::endl; else std::cout << "-" << std::endl;

    tree.remove(49, 49);
    tree.insert(50, 0);
    tree.displayTree();
  }

  return 0;
}
