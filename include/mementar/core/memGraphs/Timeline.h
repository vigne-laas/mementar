#ifndef MEMENTAR_TIMELINE_H
#define MEMENTAR_TIMELINE_H

#include "mementar/core/memGraphs/Graphs/FactGraph.h"
#include "mementar/core/memGraphs/Graphs/ActionGraph.h"

namespace mementar {

class Timeline
{
public:
  Timeline() : actions(&facts) {}

  FactGraph facts;
  ActionGraph actions;
private:
};

} // namespace mementar

#endif // MEMENTAR_TIMELINE_H
