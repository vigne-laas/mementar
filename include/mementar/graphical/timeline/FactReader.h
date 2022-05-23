#ifndef MEMENTAR_EVENTREADER_H
#define MEMENTAR_EVENTREADER_H

#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/core/core_c.h>

#include "mementar/core/memGraphs/Graphs/FactGraph.h"

namespace mementar {

struct fact_t
{
  fact_t(SoftPoint::Ttime time) : time_point(time) {}

  std::string data;
  SoftPoint time_point;
};

class FactReader
{
public:

  void read(FactGraph* graph, CvFont* font);

  std::vector<fact_t> facts;
  size_t max_text_size_;

private:
  void getTextSize(const std::string& txt, CvFont* font);
};

} // namespace mementar

#endif // MEMENTAR_EVENTREADER_H
