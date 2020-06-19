#ifndef MEMENTAR_BTREELEAF_H
#define MEMENTAR_BTREELEAF_H

#include <vector>

#include "mementar/core/memGraphs/DoublyLinkedList/DllNode.h"

namespace mementar
{

template<typename Tkey, typename Tdata, typename Tnode>
class BtreeLeafNode;

template<typename Tkey, typename Tdata, typename Tnode = DllNode<Tdata>>
class BtreeLeaf : public Tnode
{
  static_assert(std::is_base_of<DllNode<Tdata>,Tnode>::value, "Tnode must be derived from DllNode");
public:
  BtreeLeaf(const Tkey& key, const Tdata& data) : Tnode(data)
  {
    mother_ = nullptr;

    key_ = key;
  }

  ~BtreeLeaf() { }

  bool operator==(const Tkey& other) { return key_ == other; }
  bool operator>(const Tkey& other) { return key_ > other; }
  bool operator<(const Tkey& other) { return key_ < other; }
  bool operator>=(const Tkey& other) { return ((key_ > other) || (key_ == other)); }
  bool operator<=(const Tkey& other) { return ((key_ < other) || (key_ == other)); }

  bool operator==(const BtreeLeaf* other) { return key_ == other->key_; }
  bool operator>(const BtreeLeaf* other) { return key_ > other->key_; }
  bool operator<(const BtreeLeaf* other) { return key_ < other->key_; }
  bool operator>=(const BtreeLeaf* other) { return ((key_ > other->key_) || (key_ == other->key_)); }
  bool operator<=(const BtreeLeaf* other) { return ((key_ < other->key_) || (key_ == other->key_)); }

  Tkey getKey() const { return key_; }

  void setMother(BtreeLeafNode<Tkey,Tdata,Tnode>* mother) { mother_ = mother; }
  BtreeLeafNode<Tkey,Tdata,Tnode>* getMother() { return mother_; }

private:
  Tkey key_;
  BtreeLeafNode<Tkey,Tdata,Tnode>* mother_;
};

} // mementar

#endif // MEMENTAR_BTREELEAF_H
