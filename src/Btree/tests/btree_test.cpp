#include <iostream>

#include "mementar/Btree/BtreeLeaf.h"
#include "mementar/Btree/Btree.h"

int main()
{
  mementar::BtreeLeaf<int, int> leaf1(0, 2);
  mementar::BtreeLeaf<int, int> leaf2(1, 3);

  if(leaf1.operator<(&leaf2))
    std::cout << "inf" << std::endl;
  else
    std::cout << "sup" << std::endl;

  mementar::Btree<int, int> tree;
  for(size_t i = 0; i < 11; i++)
  {
    std::cout << "insert " << i << " : " << i*2 << std::endl;
    tree.insert(i, i*2);
  }

  return 0;
}
