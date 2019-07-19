#include "mementar/core/LinkedBtree/LinkedBtree.h"

int main()
{
  mementar::LinkedBtree<int> tree;

  for(int i = 20; i >= 0; i--)
  {
    if(i % 2)
      tree.insert(i, new mementar::LinkedFact("max", "hasCounted", std::to_string(i-1)));
    else
      tree.insert(i, new mementar::LinkedFact("max", "hasCounted", std::to_string(i)));
  }

  auto leaf = tree.find(1);

  std::cout << "**********" << std::endl;
  if(leaf)
  {
    std::vector<mementar::LinkedFact*> datas = leaf->getData();
    if(datas.size())
    {
      mementar::LinkedFact* c = datas[0];
      while(c != nullptr)
      {
        std::cout << *c << std::endl;
        c = c->next_;
      }
    }
    else
      std::cout << "no data in the leaf" << std::endl;
  }
  else
    std::cout << "no leaf" << std::endl;

  return 0;
}
