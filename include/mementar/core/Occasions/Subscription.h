#ifndef MEMENTAR_SUBSCRIPTION_H
#define MEMENTAR_SUBSCRIPTION_H

#include <map>
#include <vector>
#include <mutex>

#include "ontologenius/OntologyManipulator.h"

#include "mementar/core/memGraphs/Branchs/types/TripletPattern.h"
#include "mementar/core/Occasions/IdManager.h"

namespace mementar
{

class Subscription
{
public:
  Subscription(OntologyManipulator* onto = nullptr) { onto_ = onto; }

  size_t subscribe(const Triplet& patern, size_t count);
  bool unsubscribe(size_t id);

  bool isFinished(size_t id);
  bool empty() { return triplet_paterns_.size() == 0; }

  std::vector<size_t> evaluate(const Triplet& triplet);

private:
  std::map<size_t, TripletPattern> triplet_paterns_;
  std::map<size_t, size_t> counts_;
  std::mutex map_mut_;

  IdManager<size_t> id_manager_;

  OntologyManipulator* onto_;

  TripletPattern getPattern(const Triplet& triplet);
  bool compareToTriplet(const TripletPattern& pattern, const Triplet& triplet);
};

} // namespace mementar

#endif // MEMENTAR_SUBSCRIPTION_H
