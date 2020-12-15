#ifndef MEMENTAR_TIMELINE_H
#define MEMENTAR_TIMELINE_H

#include "mementar/core/memGraphs/Graphs/FactGraph.h"
#include "mementar/core/memGraphs/Graphs/ActionGraph.h"

namespace mementar {

class Timeline
{
public:
  Timeline() : actions(&facts) { init_ = true; }

  FactGraph facts;
  ActionGraph actions;

  bool isInit() { return init_; }
private:
  bool init_;
};

} // namespace mementar

#endif // MEMENTAR_TIMELINE_H
