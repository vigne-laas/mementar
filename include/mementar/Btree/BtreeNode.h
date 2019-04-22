#ifndef MEMENTAR_BTREENODE_H
#define MEMENTAR_BTREENODE_H

#include <vector>

namespace mementar
{

template<typename Tkey, typename Tdata>
class BtreeNode
{
public:
  BtreeNode(size_t order = 10)
  {
    order_ = order;
    mother_ = nullptr;
  }

  virtual ~BtreeNode()
  {
    for(auto child : childs_)
      delete child;
  }

  virtual bool isLeafNode() { return false; }

  virtual BtreeLeaf<Tkey,Tdata>* insert(const Tkey& key, const Tdata& data);
  virtual bool needBalancing();
protected:
  std::vector<Tkey> keys_;
  std::vector<BtreeNode<Tkey,Tdata>*> childs_;
  BtreeNode* mother_;
  size_t order_;
};

template<typename Tkey, typename Tdata>
BtreeLeaf<Tkey,Tdata>* BtreeNode<Tkey,Tdata>::insert(const Tkey& key, const Tdata& data)
{
  BtreeLeaf<Tkey,Tdata>* res = nullptr;
  return res;
}

template<typename Tkey, typename Tdata>
bool BtreeNode<Tkey,Tdata>::needBalancing()
{
  return (childs_.size() > order_);
}

} // namespace mementar

#endif // MEMENTAR_BTREENODE_H
