#ifndef MEMENTAR_DOUBLEQUEUE_H
#define MEMENTAR_DOUBLEQUEUE_H

#include <mutex>
#include <queue>

namespace mementar
{

template <typename T>
class DoubleQueue
{
public:
  DoubleQueue() : queue_choice_(true)
  {}

  void insert(T& data)
  {
    mutex_.lock();
    if(queue_choice_ == true)
      fifo_1.push(data);
    else
      fifo_2.push(data);

    mutex_.unlock();
  }

  void insert(std::vector<T>& datas)
  {
    mutex_.lock();
    if(queue_choice_ == true)
    {
      for(auto& data : datas)
        fifo_1.push(data);
    }
    else
    {
      for(auto& data : datas)
        fifo_2.push(data);
    }
    mutex_.unlock();
  }

  std::queue<T> get()
  {
    std::queue<T> tmp;
    mutex_.lock();
    if(queue_choice_ == true)
    {
      while(!fifo_2.empty())
        fifo_2.pop();
      queue_choice_ = false;
      tmp = fifo_1;
    }
    else
    {
      while(!fifo_1.empty())
        fifo_1.pop();
      queue_choice_ = true;
      tmp = fifo_2;
    }
    mutex_.unlock();
    return tmp;
  }

  size_t size()
  {
    return fifo_1.size() + fifo_2.size();
  }

private:
  std::mutex mutex_;

  bool queue_choice_;
  std::queue<T> fifo_1;
  std::queue<T> fifo_2;
};

} // namespace mementar

#endif // MEMENTAR_DOUBLEQUEUE_H
