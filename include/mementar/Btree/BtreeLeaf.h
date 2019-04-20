#ifndef MEMENTAR_BTREELEAF_H
#define MEMENTAR_BTREELEAF_H

namespace mementar
{

template<typename Tkey, typename Tdata>
class BtreeLeaf
{
public:
  BtreeLeaf(const Tkey& key, const Tdata& data)
  {
    next_ = nullptr;
    prev_ = nullptr;

    key_ = key;
    data_ = data;
  }

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
  Tdata getData() const { return data_; }

private:
  Tkey key_;
  Tdata data_;
};

} // mementar

#endif // MEMENTAR_BTREELEAF_H
