#include "mementar/core/Occasions/Subscription.h"

namespace mementar
{

size_t Subscription::subscribe(const TripletPattern& patern, size_t count)
{
  map_mut_.lock();
  size_t id = id_manager_.getNewId();
  triplet_paterns_.insert(std::pair<size_t, TripletPattern>(id, refinePattern(patern)));
  counts_[id] = count;
  map_mut_.unlock();

  return id;
}

bool Subscription::unsubscribe(size_t id)
{
  bool res = true;
  map_mut_.lock();
  if(id_manager_.removeId(id))
  {
    triplet_paterns_.erase(id);
    counts_.erase(id);
  }
  else
    res = false;

  map_mut_.unlock();
  return res;
}

bool Subscription::isFinished(size_t id)
{
  bool res = true;

  map_mut_.lock();
  if(counts_.find(id) != counts_.end())
    res = (counts_[id] == 0);
  map_mut_.unlock();

  return res;
}

std::vector<size_t> Subscription::evaluate(const Triplet& triplet)
{
  std::vector<size_t> res;

  map_mut_.lock();
  for(auto& it : triplet_paterns_)
  {
    if(compareToTriplet(it.second, triplet))
    {
      if(counts_[it.first])
      {
        res.push_back(it.first);
        counts_[it.first]--;
      }
    }
  }
  map_mut_.unlock();

  return res;
}

TripletPattern Subscription::refinePattern(const TripletPattern& triplet)
{
  TripletPattern pattern = triplet;
  if(onto_ != nullptr)
  {
    if(pattern.isSubjectUndefined() == false)
      if(onto_->classes.getUp(triplet.subject_).size())
        pattern.setSubjectAsClass();
    if(pattern.isObjectUndefined() == false)
      if(onto_->classes.getUp(triplet.object_).size())
        pattern.setObjectAsClass();
    if(pattern.isPredicatUndefined() == false)
      if(onto_->dataProperties.getUp(triplet.predicat_).size())
        pattern.setPredicatAsDataProperty();
  }

  return pattern;
}

bool Subscription::compareToTriplet(const TripletPattern& pattern, const Triplet& triplet)
{
  if(onto_ == nullptr)
    return pattern.fit(triplet);

  if(!pattern.isOperatorUndefined())
    if(pattern.add_ != triplet.add_)
        return false;

  if(pattern.isSubjectIndividual() && !pattern.isSubjectUndefined())
  {
    if(pattern.subject_ != triplet.subject_)
      return false;
  }

  if(pattern.isObjectIndividual() && !pattern.isObjectUndefined())
  {
    if(pattern.object_ != triplet.object_)
      return false;
  }

  if(pattern.isSubjectIndividual() == false)
  {
    if(onto_->individuals.isA(triplet.subject_, pattern.subject_) == false)
      return false;
  }
  // subject match

  if(pattern.isObjectIndividual() == false)
  {
    if(onto_->individuals.isA(triplet.object_, pattern.object_) == false)
      return false;
  }
  // object match

  if((pattern.predicat_ != triplet.predicat_) && !pattern.isPredicatUndefined())
  {
    if(pattern.isPredicatObjectProperty())
    {
      if(onto_->objectProperties.isA(triplet.predicat_, pattern.predicat_) == false)
        return false;
    }
    else
    {
      if(onto_->dataProperties.isA(triplet.predicat_, pattern.predicat_) == false)
        return false;
    }
  }
  // predicat match

  return true;
}

} // namespace mementar
