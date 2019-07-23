#ifndef MEMENTAR_STAMPEDDATA_H
#define MEMENTAR_STAMPEDDATA_H

namespace mementar
{

template<typename T>
class StampedData
{
public:
  StampedData(T stamp)
  {
    stamp_ = stamp;
  }

  T getStamp() const
  {
    return stamp_;
  }

private:
  T stamp_;
};

} // namespace mementar

#endif // MEMENTAR_STAMPEDDATA_H
