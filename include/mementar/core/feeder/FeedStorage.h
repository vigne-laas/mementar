#ifndef MEMENTER_FEEDSTORAGE_H
#define MEMENTER_FEEDSTORAGE_H

#include <regex>
#include <mutex>
#include <string>
#include <queue>

#include "mementar/core/memGraphs/Branchs/types/Fact.h"

namespace mementar {

enum action_t
{
  action_add,
  action_del,
  action_commit,
  action_checkout,
  action_nop
};

struct feed_t
{
  action_t action_;
  std::experimental::optional<Fact> fact_;
  std::experimental::optional<Fact> expl_;
  bool checkout_;

  feed_t() { checkout_ = false; }
};

class FeedStorage
{
public:
  FeedStorage();

  void insert(const std::string& regex, const SoftPoint::Ttime& stamp);
  void insert(const std::string& regex, const std::string& expl_regex);
  void insert(std::vector<feed_t>& datas);
  std::queue<feed_t> get();
  size_t size() { return fifo_1.size() + fifo_2.size(); }

private:
  std::regex base_regex;
  std::mutex mutex_;

  bool queue_choice_;
  std::queue<feed_t> fifo_1;
  std::queue<feed_t> fifo_2;

  feed_t getFeed(const std::string& regex, const SoftPoint::Ttime& stamp = SoftPoint::default_time);
};

} // namespace mementar

#endif // MEMENTER_FEEDSTORAGE_H
