#ifndef MEMENTAR_TIMELINEDRAWER_H
#define MEMENTAR_TIMELINEDRAWER_H

#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui_c.h>

#include "mementar/core/memGraphs/Timeline.h"
#include "mementar/graphical/timeline/ActionReader.h"
#include "mementar/graphical/timeline/FactReader.h"

namespace mementar {

class TimelineDrawer
{
public:

  bool draw(const std::string& file_name, Timeline* timeline);
private:
  IplImage* image_;

  void drawVector(size_t start, size_t end, size_t pose, CvFont* font);
  void drawAction(const action_t& action, size_t line_pose, size_t max_level, size_t start_time, size_t end_time, CvFont* font);
  void drawEvent(const fact_t& event, size_t line_pose, size_t start_time, CvFont* font);

  size_t getTextSize(const std::string& txt, CvFont* font);
  void drawElipseStart(size_t x, size_t y, CvScalar color);
  void drawElipseEnd(size_t x, size_t y, CvScalar color);

  CvScalar ScalarHSV2BGR(float H, float S, float V);;
};

} // namespace mementar

#endif // MEMENTAR_TIMELINEDRAWER_H
