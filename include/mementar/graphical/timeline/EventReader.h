#ifndef MEMENTAR_EVENTREADER_H
#define MEMENTAR_EVENTREADER_H

#include <string>
#include <vector>

#include <opencv2/imgproc/imgproc.hpp>

#include "mementar/core/memGraphs/Graphs/EventGraph.h"

namespace mementar {

struct event_t
{
  std::string data;
  size_t time_point;
};

class EventReader
{
public:

  void read(EventGraph* graph, CvFont* font);

  std::vector<event_t> events;
  size_t max_text_size_;

private:
  void getTextSize(const std::string& txt, CvFont* font);
};

} // namespace mementar

#endif // MEMENTAR_EVENTREADER_H
