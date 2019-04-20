#include <iostream>

#include "mementar/Btree/BtreeLeaf.h"

int main()
{
  mementar::BtreeLeaf<int, int> leaf1(0, 2);
  mementar::BtreeLeaf<int, int> leaf2(1, 3);

  if(leaf1.operator<(&leaf2))
    std::cout << "inf" << std::endl;
  else
    std::cout << "sup" << std::endl;

  return 0;
}
