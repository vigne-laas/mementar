#ifndef MEMENTAR_BTREELEAF_H
#define MEMENTAR_BTREELEAF_H

#include <vector>

namespace mementar {

template<typename Tkey, typename Tleaf>
class BtreeNode;

template<typename Tkey, typename Tleaf>
class BtreeLeafBase : public Tleaf
{
  using Tdata = typename Tleaf::DataType;
public:
  BtreeLeafBase(const Tkey& key)
  {
    mother_ = nullptr;

    key_ = key;
  }

  bool operator==(const BtreeLeafBase& other) { return key_ == other.key_; }
  bool operator>(const BtreeLeafBase& other) { return key_ > other.key_; }
  bool operator<(const BtreeLeafBase& other) { return key_ < other.key_; }
  bool operator>=(const BtreeLeafBase& other) { return ((key_ > other.key_) || (key_ == other.key_)); }
  bool operator<=(const BtreeLeafBase& other) { return ((key_ < other.key_) || (key_ == other.key_)); }

  Tkey getKey() const { return key_; }

  void setMother(BtreeNode<Tkey,Tleaf>* mother) { mother_ = mother; }
  BtreeNode<Tkey,Tleaf>* getMother() { return mother_; }

private:
  Tkey key_;
  BtreeNode<Tkey,Tleaf>* mother_;
};

} // namespace mementar

#endif // MEMENTAR_BTREELEAF_H
