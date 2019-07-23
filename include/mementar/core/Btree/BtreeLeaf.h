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
  BtreeLeaf(Tdata* data)
  {
    next_ = nullptr;
    prev_ = nullptr;
    mother_ = nullptr;

    key_ = data->getStamp();
    data_.push_back(data);
  }

  ~BtreeLeaf()
  {
    for(auto data : data_)
      delete data;
    data_.clear();
  }

  void push_back(Tdata* data) { data_.push_back(data); }
  void remove(const Tdata& data);

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
  std::vector<Tdata*> getData() const { return data_; }
  void getData(std::vector<Tdata*>& data) { data = data_; }
  Tdata* getData(const Tdata& data);

  void setMother(BtreeLeafNode<Tkey,Tdata>* mother) { mother_ = mother; }
  BtreeLeafNode<Tkey,Tdata>* getMother() { return mother_; }

private:
  Tkey key_;
  std::vector<Tdata*> data_;
  BtreeLeafNode<Tkey,Tdata>* mother_;
};

template<typename Tkey, typename Tdata>
void BtreeLeaf<Tkey,Tdata>::remove(const Tdata& data)
{
  for(size_t i = 0; i < data_.size();)
  {
    if(data_[i]->operator==(data))
    {
      delete data_[i];
      data_.erase(data_.begin() + i);
    }
    else
      i++;
  }
}

template<typename Tkey, typename Tdata>
Tdata* BtreeLeaf<Tkey,Tdata>::getData(const Tdata& data)
{
  for(size_t i = 0; i < data_.size(); i++)
  {
    if(data_[i]->operator==(data))
      return data_[i];
  }
  return nullptr;
}

} // mementar

#endif // MEMENTAR_BTREELEAF_H
