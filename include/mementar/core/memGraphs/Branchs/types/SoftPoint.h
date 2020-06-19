#ifndef MEMENTAR_SOFTPOINT_H
#define MEMENTAR_SOFTPOINT_H

#include <experimental/optional>
#include <cstdlib>

namespace mementar {

class SoftPoint
{
public:
  SoftPoint(size_t t_start, std::experimental::optional<size_t> t_end = std::experimental::nullopt)
  {
    t_start_ = t_start;
    t_end_ = t_end;
    t_ = t_start_ + (t_end_.value_or(t_start_) - t_start_) / 2;
  }

  SoftPoint(const SoftPoint& other)
  {
    t_start_ = other.t_start_;
    t_end_ = other.t_end_;
    t_ = t_start_ + (t_end_.value_or(t_start_) - t_start_) / 2;
  }

  bool isInstantaneous() const { return t_end_ == std::experimental::nullopt; }
  size_t getTime() const { return t_; }
  size_t getTimeStart() const { return t_start_; }
  size_t getTimeEnd() const { return t_end_.value_or(t_start_); }
  size_t getTransitionDuration() const { return t_end_.value_or(t_start_) - t_start_; }

  std::string toString() const { return "[" + std::to_string(t_start_) + std::string(t_end_ ? "," + std::to_string(t_end_.value()) : "") + "]"; }

  typedef size_t Ttime;

protected:
  Ttime t_start_;
  std::experimental::optional<Ttime> t_end_;
  Ttime t_;
};

} // namespace mementar

#endif // MEMENTAR_SOFTPOINT_H
