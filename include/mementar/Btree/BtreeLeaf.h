#ifndef MEMENTAR_BTREELEAF_H
#define MEMENTAR_BTREELEAF_H

#include <vector>

namespace mementar
{

template<typename Tkey, typename Tdata>
class BtreeLeafNode;

template<typename Tkey, typename Tdata>
class BtreeLeaf
{
public:
  BtreeLeaf(const Tkey& key, const Tdata& data)
  {
    next_ = nullptr;
    prev_ = nullptr;
    mother_ = nullptr;

    key_ = key;
    data_.push_back(data);
  }

  ~BtreeLeaf() {}

  void push_back(const Tdata& data) { data_.push_back(data); }

  BtreeLeaf* next_;
  BtreeLeaf* prev_;

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
  std::vector<Tdata> getData() const { return data_; }
  void getData(std::vector<Tdata>& data) { data = data_; }

  void setMother(BtreeLeafNode<Tkey,Tdata>* mother) { mother_ = mother; }
  BtreeLeafNode<Tkey,Tdata>* getMother() { return mother_; }

private:
  Tkey key_;
  std::vector<Tdata> data_;
  BtreeLeafNode<Tkey,Tdata>* mother_;
};

} // mementar

#endif // MEMENTAR_BTREELEAF_H
