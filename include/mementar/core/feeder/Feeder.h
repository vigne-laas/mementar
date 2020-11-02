#ifndef MEMENTAR_FEEDER_H
#define MEMENTAR_FEEDER_H

#include "mementar/core/feeder/FeedStorage.h"
#include "mementar/core/feeder/IdGenerator.h"
//#include "ontologenius/core/feeder/Versionor.h"

#include <functional>

namespace mementar {

class Timeline;

class Feeder
{
public:
  Feeder(Timeline* timeline = nullptr);

  void store(const std::string& feed, const SoftPoint::Ttime& stamp) { feed_storage_.insert(feed, stamp); }
  void store(const std::string& feed, const std::string& expl) { feed_storage_.insert(feed, expl); }
  bool run();
  void link(Timeline* timeline) {timeline_ = timeline; }
  void setCallback(const std::function<void(const Triplet&)>& callback) { callback_ = callback; }

  std::vector<std::string> getNotifications()
  {
    auto tmp = std::move(notifications_);
    notifications_.clear();
    return tmp;
  }

  //void activateVersionning(bool activated) { versionor_.activate(activated); }
  //void exportToXml(const std::string& path) { versionor_.exportToXml(path); }

  size_t size() { return feed_storage_.size(); }

private:
  FeedStorage feed_storage_;
  IdGenerator id_generator_;
  //Versionor versionor_;
  Timeline* timeline_;
  std::function<void(const Triplet&)> callback_;

  // Here the notifications are about miss formed queries
  std::vector<std::string> notifications_;

  void defaultCallback(const Triplet&) {}
};

} // namespace mementar

#endif // MEMENTAR_FEEDER_H
