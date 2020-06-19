#include "mementar/core/memGraphs/Branchs/types/Action.h"
#include "mementar/core/memGraphs/Branchs/types/Event.h"

#include "mementar/core/memGraphs/Branchs/ContextualizedEvent.h"

#include "mementar/core/memGraphs/Graphs/EventGraph.h"
#include "mementar/core/memGraphs/Graphs/ActionGraph.h"

#include <iostream>

void printNext(mementar::ContextualizedEvent* evt)
{
  auto nexts = evt->getNextDllData();
  for(auto n : nexts)
    std::cout << static_cast<mementar::ContextualizedEvent*>(n)->toString() << std::endl;

  if(nexts.size())
    printNext( static_cast<mementar::ContextualizedEvent*>(nexts[0]));
}

void print(mementar::ContextualizedEvent* evt)
{
  std::cout << evt->toString() << std::endl;

  auto nexts = evt->getNextDllData();
  for(auto n : nexts)
    std::cout <<  static_cast<mementar::ContextualizedEvent*>(n)->toString() << std::endl;

  if(nexts.size())
    printNext( static_cast<mementar::ContextualizedEvent*>(nexts[0]));
}

void printEventList(mementar::ContextualizedEvent* evt)
{
  if(evt)
  {
    std::cout << evt->toString() << std::endl;
    printEventList(dynamic_cast<mementar::ContextualizedEvent*>(evt->getNextEllElement()));
  }
}

int main()
{
  mementar::EventGraph event_graph;
  mementar::ActionGraph action_graph(&event_graph);

  mementar::Event e1("cube12|isOn|Table_1", 2,4);
  mementar::Event e2("cube12|isOn|Table_2", 8);

  action_graph.add(new mementar::Action("pick_1", mementar::SoftPoint(1, 3), 7));
  action_graph.add(new mementar::Action("place_1", 7, mementar::SoftPoint(9, 10)));
  action_graph.add(new mementar::Action("pick_2", 12));

  auto pending = action_graph.getPendingSafe();
  for(auto action : pending)
    std::cout << action->getName() << " is a pending action" << std::endl;

  action_graph.setEnd("pick_2", 15);
  action_graph.setEnd("pick_1", 17);

  event_graph.add(new mementar::ContextualizedEvent("ce3", e1));
  event_graph.add(new mementar::ContextualizedEvent("ce4", e2));

  for(size_t i = 10; i < 20; i++)
    event_graph.add(new mementar::ContextualizedEvent("ce" + std::to_string(i), mementar::Event("cube" + std::to_string(i) + "|isOn|Table_3", i)));

  auto actions = action_graph.getSafe();
  for(auto action : actions)
    std::cout << action->getName() << " D=" << action->getDuration() << " Dmin=" << action->getMinDuration() << " Dmax=" << action->getMaxDuration() << std::endl;

  auto events = event_graph.getSafe();
  for(auto event : events)
    std::cout << event->toString() << std::endl;

  auto timeline = event_graph.getTimeline();
  timeline->display();

  std::cout << "********" << std::endl;
  auto first_evt = event_graph.findBranch("pick_1_start");
  print(first_evt);

  std::cout << "********" << std::endl;
  auto cube12_ison_evt = event_graph.findBranch("ce3");
  printEventList(cube12_ison_evt);

  return 0;
}
