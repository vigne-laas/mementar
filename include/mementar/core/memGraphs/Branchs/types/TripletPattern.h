#ifndef MEMENTAR_TRIPLETPATTERN_H
#define MEMENTAR_TRIPLETPATTERN_H

#include <string>
#include <ostream>

#include "mementar/core/memGraphs/Branchs/types/Triplet.h"

namespace mementar {

class TripletPattern : public Triplet
{
public:
  TripletPattern(const std::string& subject, const std::string& predicat, const std::string& object, bool add) : Triplet(subject, predicat, object, add)
  {
    init();
  }

  TripletPattern(const std::string& subject, const std::string& predicat, const std::string& object) : Triplet(subject, predicat, object, true)
  {
    init();
    operator_is_undefined_ = true;
  }

  TripletPattern(const std::string& str_triplet, bool add) : Triplet(str_triplet, add)
  {
    init();
  }

  TripletPattern(const std::string& str_triplet) : Triplet(str_triplet, true)
  {
    init();
    operator_is_undefined_ = true;
  }

  TripletPattern(const Triplet& triplet) : Triplet(triplet)
  {
    init();
  }

  TripletPattern(const TripletPattern& other) : Triplet(other)
  {
    subject_is_indiv_ = other.subject_is_indiv_;
    object_is_indiv_ = other.object_is_indiv_;
    predicat_is_object_property_ = other.predicat_is_object_property_;

    subject_is_undefined_ = other.subject_is_undefined_;
    object_is_undefined_ = other.object_is_undefined_;
    predicat_is_undefined_ = other.predicat_is_undefined_;
    operator_is_undefined_  =other.operator_is_undefined_;
  }

  static TripletPattern deserialize(const std::string& str)
  {
    if(std::regex_match(str, match, regex))
    {
      if(match[1].str() == "?")
        return TripletPattern(match[2].str(), match[3].str(), match[4].str());
      else
        return TripletPattern(match[2].str(), match[3].str(), match[4].str(), match[1].str() == "A");
    }
    else if(std::regex_match(str, match, regex2))
    {
      if(match[1].str() == "?")
        return Triplet(match[2].str(), match[3].str(), match[4].str());
      else
        return Triplet(match[2].str(), match[3].str(), match[4].str(), (match[1].str() == "ADD") || (match[1].str() == "add"));
    }
    else
      return Triplet();
  }

  static std::string serialize(const TripletPattern& pattern)
  {
    return Triplet::serialize(pattern);
  }

  static std::string serialize(const TripletPattern* pattern)
  {
    return Triplet::serialize(pattern);
  }

  bool fit(const Triplet& other) const
  {
    return ( ((add_ == other.add_) || object_is_undefined_)
            && ((subject_ == other.subject_) || subject_is_undefined_)
            && ((predicat_ == other.predicat_) || predicat_is_undefined_)
            && ((object_ == other.object_) || subject_is_undefined_));
  }

  bool operator==(const Triplet& other) const = delete;
  bool operator==(const Triplet* other) const = delete;

  void setSubjectAsClass() { subject_is_indiv_ = false; }
  void setSubjectAsIndividual() { subject_is_indiv_ = true; }
  void setObjectAsClass() { object_is_indiv_ = false; }
  void setObjectAsIndividual() { object_is_indiv_ = true; }
  void setPredicatAsDataProperty() { predicat_is_object_property_ = false; }
  void setPredicatAsObjectProperty() { predicat_is_object_property_ = true; }

  bool isSubjectClass () const { return !subject_is_indiv_; }
  bool isSubjectIndividual() const { return subject_is_indiv_; }
  bool isObjectClass() const { return !object_is_indiv_; }
  bool isObjectIndividual() const { return object_is_indiv_; }
  bool isPredicatDataProperty() const { return !predicat_is_object_property_; }
  bool isPredicatObjectProperty() const { return predicat_is_object_property_; }

  bool isOperatorUndefined() const { return operator_is_undefined_; }
  bool isSubjectUndefined() const { return subject_is_undefined_; }
  bool isObjectUndefined() const { return object_is_undefined_; }
  bool isPredicatUndefined() const { return predicat_is_undefined_; }

protected:
  bool subject_is_indiv_;
  bool object_is_indiv_;
  bool predicat_is_object_property_;

  bool operator_is_undefined_;
  bool subject_is_undefined_;
  bool object_is_undefined_;
  bool predicat_is_undefined_;

private:
  void init()
  {
    subject_is_indiv_ = true;
    object_is_indiv_ = true;
    predicat_is_object_property_ = true;

    subject_is_undefined_ = (this->subject_ == "?");
    object_is_undefined_ = (this->object_ == "?");
    predicat_is_undefined_ = (this->predicat_ == "?");
    operator_is_undefined_ = false;
  }
};

} // namespace mementar

#endif // MEMENTAR_TRIPLETPATTERN_H
