#ifndef MEMENTAR_DLLNODE_H
#define MEMENTAR_DLLNODE_H

#include <vector>
#include <string>
#include <iostream>

namespace mementar {

template<typename Tdata>
class DllNode
{
std::false_type is_dllnode_impl(...);
template <typename T>
std::true_type is_dllnode_impl(DllNode<T>*);
template <typename U>
using is_dllnode = decltype(is_dllnode_impl(std::declval<U*>()));

public:
  DllNode()
  {
    prev_ = nullptr;
    next_ = nullptr;
  }

  virtual ~DllNode() {}

  virtual void push_back(const Tdata& data) { payload_.push_back(data); }
  virtual void remove(const Tdata& data)
  {
    for(size_t i = 0; i < payload_.size();)
    {
      if(payload_[i] == data)
        payload_.erase(payload_.begin() + i);
      else
        i++;
    }
  }

  std::vector<Tdata> getData() const { return payload_; }
  void getData(std::vector<Tdata>& data) { data = payload_; }

  inline DllNode* getPreviousNode() { return prev_; }
  inline DllNode* getNextNode() { return next_; }

  virtual void setPreviousNode(DllNode* prev) { prev_ = prev; }
  virtual void setNextNode(DllNode* next) { next_ = next; }

protected:
  DllNode* prev_;
  DllNode* next_;
  std::vector<Tdata> payload_;
};

} // namespace mementar

#endif // MEMENTAR_DLLNODE_H
