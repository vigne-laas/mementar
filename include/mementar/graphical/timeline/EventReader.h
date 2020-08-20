#ifndef MEMENTAR_EVENTREADER_H
#define MEMENTAR_EVENTREADER_H

#include <string>
#include <vector>

#include <opencv2/imgproc/imgproc.hpp>

#include "mementar/core/memGraphs/Graphs/FactGraph.h"

namespace mementar {

struct event_t
{
  event_t(SoftPoint::Ttime time) : time_point(time) {}

  std::string data;
  SoftPoint time_point;
};

class EventReader
{
public:

  void read(FactGraph* graph, CvFont* font);

  std::vector<event_t> events;
  size_t max_text_size_;

private:
  void getTextSize(const std::string& txt, CvFont* font);
};

} // namespace mementar

#endif // MEMENTAR_EVENTREADER_H
