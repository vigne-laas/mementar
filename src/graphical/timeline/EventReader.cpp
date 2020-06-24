#include "mementar/graphical/timeline/EventReader.h"

namespace mementar {

void EventReader::read(EventGraph* graph, CvFont* font)
{
  max_text_size_ = 0;

  auto tree = graph->getTimeline();
  auto node = tree->getFirst();

  while(node != nullptr)
  {
    event_t group_evt(node->getKey());
    for(auto evt : node->getData())
    {
      if(dynamic_cast<ContextualizedEvent*>(evt)->isPartOfAction() == false)
      {
        auto event = dynamic_cast<ContextualizedEvent*>(evt);
        group_evt.data += (group_evt.data == "" ? "" : " -- ") + event->Fact::toString();
        if(event->getTransitionDuration() > group_evt.time_point.getTransitionDuration())
          group_evt.time_point = SoftPoint(event);
      }
    }

    if(group_evt.data != "")
    {
      events.push_back(group_evt);
      getTextSize(group_evt.data, font);
    }

    node = dynamic_cast<BtreeLeaf<SoftPoint::Ttime, EllElement*, EllNode>*>(node->getNextNode());
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
