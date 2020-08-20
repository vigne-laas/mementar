#include "mementar/graphical/timeline/EventReader.h"

namespace mementar {

void EventReader::read(FactGraph* graph, CvFont* font)
{
  max_text_size_ = 0;

  auto tree = graph->getTimeline();
  auto node = static_cast<BplusLeaf<SoftPoint::Ttime, ContextualizedFact*>*>(tree->getFirst());

  while(node != nullptr)
  {
    event_t group_fact(node->getData()[0]->getTime());
    for(auto fact : node->getData())
    {
      if(fact->isPartOfAction() == false)
      {
        group_fact.data += (group_fact.data == "" ? "" : " -- ") + fact->Triplet::toString();
        if(fact->getTransitionDuration() > group_fact.time_point.getTransitionDuration())
          group_fact.time_point = SoftPoint(fact);
      }
    }

    if(group_fact.data != "")
    {
      events.push_back(group_fact);
      getTextSize(group_fact.data, font);
    }

    node = node->getNextLeaf();
  }
}

void EventReader::getTextSize(const std::string& txt, CvFont* font)
{
  CvSize size;
  int baseline;
  cvGetTextSize(txt.c_str(), font, &size, &baseline);
  size_t text_width = size.width;
  if(text_width > max_text_size_)
    max_text_size_ = text_width;
}

} // namespace mementar
