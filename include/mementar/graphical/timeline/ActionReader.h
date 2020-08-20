#ifndef MEMENTAR_ACTIONREADER_H
#define MEMENTAR_ACTIONREADER_H

#include <string>
#include <map>
#include <experimental/optional>

#include <opencv2/imgproc/imgproc.hpp>

#include "mementar/core/memGraphs/Graphs/FactGraph.h"

namespace mementar {

struct action_t
{
  action_t(const SoftPoint& point) : start(point) {}

  std::string name;
  SoftPoint start;
  std::experimental::optional<SoftPoint> end;

  size_t level;
};

class ActionReader
{
public:
  ActionReader();

  void read(FactGraph* graph, CvFont* font);

  std::map<std::string, action_t> actions_;
  size_t max_level_;
  size_t max_text_size_;

private:
  std::vector<size_t> levels_;

  action_t getAction(Action* action);
  void closeAction(Action* action);

  size_t getMinLevel();
  void getTextSize(const std::string& txt, CvFont* font);
};

} // namespace mementar

#endif // MEMENTAR_ACTIONREADER_H
