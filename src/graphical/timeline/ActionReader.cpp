#include "mementar/graphical/timeline/ActionReader.h"

namespace mementar {

ActionReader::ActionReader()
{
  actions_.clear();
  levels_.clear();
  max_level_ = 0;
  max_text_size_ = 0;
}

void ActionReader::read(FactGraph* graph, CvFont* font)
{
  auto tree = graph->getTimeline();
  auto node = static_cast<BplusLeaf<SoftPoint::Ttime, ContextualizedFact*>*>(tree->getFirst());

  while(node != nullptr)
  {
    for(auto fact : node->getData())
    {
      if(fact->isPartOfAction())
      {
        auto action = fact->getActionPart();
        if(actions_.find(action->getValue()) == actions_.end())
        {
          action_t act = getAction(action);
          actions_.insert(std::pair<std::string, action_t>(act.name, act));
          getTextSize(act.name, font);
        }
        else
          closeAction(action);
      }
    }

    node = node->getNextLeaf();
  }
}

action_t ActionReader::getAction(Action* action)
{
  action_t new_action(*action->getStartFact());
  new_action.name = action->getValue();
  new_action.level = getMinLevel();

  levels_.push_back(new_action.level);
  if(new_action.level > max_level_)
    max_level_ = new_action.level;

  return new_action;
}

void ActionReader::closeAction(Action* action)
{
  auto act_it = actions_.find(action->getValue());
  auto level_it = std::find(levels_.begin(), levels_.end(), act_it->second.level);
  levels_.erase(level_it);
  act_it->second.end = SoftPoint(*action->getEndFact());
}

size_t ActionReader::getMinLevel()
{
  for(size_t l = 1; ; l++)
  {
    if(std::find(levels_.begin(), levels_.end(), l) == levels_.end())
      return l;
  }
}

void ActionReader::getTextSize(const std::string& txt, CvFont* font)
{
  CvSize size;
  int baseline;
  cvGetTextSize(txt.c_str(), font, &size, &baseline);
  size_t text_width = size.width;
  if(text_width > max_text_size_)
    max_text_size_ = text_width;
}

} // namespace mementar
