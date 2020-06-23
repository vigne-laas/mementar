#include "mementar/core/memGraphs/Timeline.h"

#include "mementar/graphical/timeline/ActionReader.h"
#include "mementar/graphical/timeline/EventReader.h"
#include "mementar/graphical/timeline/TimelineDrawer.h"

mementar::Timeline* getTimeline()
{
  mementar::Timeline* timeline = new mementar::Timeline();

  timeline->actions.add(new mementar::Action("blue_pick_1", mementar::SoftPoint(35, 36), 37));
  timeline->actions.add(new mementar::Action("blue_place_1", 37, mementar::SoftPoint(41, 42)));
  timeline->actions.add(new mementar::Action("monitor_1", 43, 51));
  timeline->actions.add(new mementar::Action("explain_goal_1", 69, 72));
  timeline->actions.add(new mementar::Action("explain_action_1", 74, 76));
  timeline->actions.add(new mementar::Action("rob_pick_1", 76, 91));
  timeline->actions.add(new mementar::Action("rob_ack_1", 91, 91));
  timeline->actions.add(new mementar::Action("explain_action_2", 95, 98));
  timeline->actions.add(new mementar::Action("rob_drop_1", 100, 110));
  timeline->actions.add(new mementar::Action("rob_ack_2", 110, 110));
  timeline->actions.add(new mementar::Action("explain_loc_1", 116, 119));
  timeline->actions.add(new mementar::Action("explain_action_3", 131, 134));
  timeline->actions.add(new mementar::Action("green_pick_1", mementar::SoftPoint(132, 133), mementar::SoftPoint(134, 135)));
  timeline->actions.add(new mementar::Action("explain_action_4", 142, 144));
  timeline->actions.add(new mementar::Action("green_place_1", mementar::SoftPoint(143, 144), 145));
  timeline->actions.add(new mementar::Action("rob_ack_3", 148, 149));
  timeline->actions.add(new mementar::Action("explain_action_5", 157, 159));
  timeline->actions.add(new mementar::Action("rob_pick_2", 158, 163));
  timeline->actions.add(new mementar::Action("rob_ack_4", 165, 165));
  timeline->actions.add(new mementar::Action("explain_action_6", 167, 170));
  timeline->actions.add(new mementar::Action("rob_drop_2", 170, 174));
  timeline->actions.add(new mementar::Action("rob_ack_5", 176, 176));

  timeline->events.add(new mementar::ContextualizedEvent("e0", mementar::Event("pr2|isInRoom|expe_room", 4)));
  timeline->events.add(new mementar::ContextualizedEvent("e1", mementar::Event("grey_box|isOn|Table_1", 5)));
  timeline->events.add(new mementar::ContextualizedEvent("e2", mementar::Event("tape_1|isOn|Table_1", 5)));
  timeline->events.add(new mementar::ContextualizedEvent("e3", mementar::Event("tape_2|isIn|storage_area", 5)));
  timeline->events.add(new mementar::ContextualizedEvent("e4", mementar::Event("hum_green|isInRoom|expe_room", 6,7)));
  timeline->events.add(new mementar::ContextualizedEvent("e5", mementar::Event(mementar::Fact("hum_green|isInRoom|expe_room", false), mementar::SoftPoint(25,26))));
  timeline->events.add(new mementar::ContextualizedEvent("e6", mementar::Event("hum_blue|isInRoom|expe_room", 30,31)));
  timeline->events.add(new mementar::ContextualizedEvent("e7", mementar::Event("tape_1|isInHand|hum_blue", 37)));
  timeline->events.add(new mementar::ContextualizedEvent("e8", mementar::Event(mementar::Fact("tape_1|isOn|Table_1", false), 37)));
  timeline->events.add(new mementar::ContextualizedEvent("e9", mementar::Event("tape_1|isOn|Table_1", 38, 42)));
  timeline->events.add(new mementar::ContextualizedEvent("e10", mementar::Event("tape_1|isBehind|grey_box", 38)));
  timeline->events.add(new mementar::ContextualizedEvent("e11", mementar::Event(mementar::Fact("tape_1|isInHand|hum_blue", false), 42)));
  timeline->events.add(new mementar::ContextualizedEvent("e12", mementar::Event(mementar::Fact("hum_blue|isInRoom|expe_room", false), mementar::SoftPoint(51,52))));
  timeline->events.add(new mementar::ContextualizedEvent("e13", mementar::Event("hum_green|isInRoom|expe_room", 53)));
  timeline->events.add(new mementar::ContextualizedEvent("e14", mementar::Event("tape_2|isInGripper|pr2", 86)));
  timeline->events.add(new mementar::ContextualizedEvent("e15", mementar::Event(mementar::Fact("tape_2|isIn|storage_area", false), 101)));
  timeline->events.add(new mementar::ContextualizedEvent("e16", mementar::Event(mementar::Fact("tape_2|isInGripper|pr2", false), 107)));
  timeline->events.add(new mementar::ContextualizedEvent("e17", mementar::Event("tape_2|isIn|pink_box", 107,110)));
  timeline->events.add(new mementar::ContextualizedEvent("e18", mementar::Event("tape_1|isInHand|hum_green", 133, 134)));
  timeline->events.add(new mementar::ContextualizedEvent("e19", mementar::Event(mementar::Fact("tape_1|isOn|Table_1", false), 134)));
  timeline->events.add(new mementar::ContextualizedEvent("e20", mementar::Event(mementar::Fact("tape_1|isBehind|grey_box", false), 134)));
  timeline->events.add(new mementar::ContextualizedEvent("e21", mementar::Event("tape_1|isIn|storage_area", 145)));
  timeline->events.add(new mementar::ContextualizedEvent("e23", mementar::Event(mementar::Fact("tape_1|isInHand|hum_green", false), 145)));
  timeline->events.add(new mementar::ContextualizedEvent("e24", mementar::Event("tape_1|isInGripper|pr2", 162)));
  timeline->events.add(new mementar::ContextualizedEvent("e25", mementar::Event(mementar::Fact("tape_1|isIn|storage_area", false), 170)));
  timeline->events.add(new mementar::ContextualizedEvent("e26", mementar::Event(mementar::Fact("tape_1|isInGripper|pr2", false), 173)));
  timeline->events.add(new mementar::ContextualizedEvent("e27", mementar::Event("tape_1|isIn|pink_box", 173,175)));

  return timeline;
}


/* HUMAN GREEN*/
/*mementar::Timeline* getTimeline()
{
  mementar::Timeline* timeline = new mementar::Timeline();

  timeline->actions.add(new mementar::Action("explain_goal_1", 69, 72));
  timeline->actions.add(new mementar::Action("explain_action_1", 74, 76));
  timeline->actions.add(new mementar::Action("rob_pick_1", 76, 91));
  timeline->actions.add(new mementar::Action("rob_ack_1", 91, 91));
  timeline->actions.add(new mementar::Action("explain_action_2", 95, 98));
  timeline->actions.add(new mementar::Action("rob_drop_1", 100, 110));
  timeline->actions.add(new mementar::Action("rob_ack_2", 110, 110));
  timeline->actions.add(new mementar::Action("explain_loc_1", 116, 119));
  timeline->actions.add(new mementar::Action("explain_action_3", 131, 134));
  timeline->actions.add(new mementar::Action("green_pick_1", mementar::SoftPoint(132, 133), mementar::SoftPoint(134, 135)));
  timeline->actions.add(new mementar::Action("explain_action_4", 142, 144));
  timeline->actions.add(new mementar::Action("green_place_1", mementar::SoftPoint(143, 144), 145));
  timeline->actions.add(new mementar::Action("rob_ack_3", 148, 149));
  timeline->actions.add(new mementar::Action("explain_action_5", 157, 159));
  timeline->actions.add(new mementar::Action("rob_pick_2", 158, 163));
  timeline->actions.add(new mementar::Action("rob_ack_4", 165, 165));
  timeline->actions.add(new mementar::Action("explain_action_6", 167, 170));
  timeline->actions.add(new mementar::Action("rob_drop_2", 170, 174));
  timeline->actions.add(new mementar::Action("rob_ack_5", 176, 176));

  timeline->events.add(new mementar::ContextualizedEvent("e0", mementar::Event("pr2|isInRoom|expe_room", 6,7)));
  timeline->events.add(new mementar::ContextualizedEvent("e1", mementar::Event("grey_box|isOn|Table_1", 7, 8)));
  timeline->events.add(new mementar::ContextualizedEvent("e2", mementar::Event("tape_1|isOn|Table_1", 7, 8)));
  timeline->events.add(new mementar::ContextualizedEvent("e3", mementar::Event("tape_2|isIn|storage_area", 7.8)));
  timeline->events.add(new mementar::ContextualizedEvent("e4", mementar::Event("hum_green|isInRoom|expe_room", 6,7)));
  timeline->events.add(new mementar::ContextualizedEvent("e5", mementar::Event(mementar::Fact("hum_green|isInRoom|expe_room", false), mementar::SoftPoint(25,26))));

  timeline->events.add(new mementar::ContextualizedEvent("e13", mementar::Event("hum_green|isInRoom|expe_room", 53)));
  timeline->events.add(new mementar::ContextualizedEvent("e8", mementar::Event(mementar::Fact("tape_1|isOn|Table_1", false), 54)));
  timeline->events.add(new mementar::ContextualizedEvent("e14", mementar::Event("tape_2|isInGripper|pr2", 86)));
  timeline->events.add(new mementar::ContextualizedEvent("e15", mementar::Event(mementar::Fact("tape_2|isIn|storage_area", false), 101)));
  timeline->events.add(new mementar::ContextualizedEvent("e16", mementar::Event(mementar::Fact("tape_2|isInGripper|pr2", false), 107)));
  timeline->events.add(new mementar::ContextualizedEvent("e17", mementar::Event("tape_2|isIn|pink_box", 107,110)));

  timeline->events.add(new mementar::ContextualizedEvent("e9", mementar::Event("tape_1|isOn|Table_1", 117, 119)));
  timeline->events.add(new mementar::ContextualizedEvent("e10", mementar::Event("tape_1|isBehind|grey_box", 117, 119)));

  timeline->events.add(new mementar::ContextualizedEvent("e18", mementar::Event("tape_1|isInHand|hum_green", 133, 134)));
  timeline->events.add(new mementar::ContextualizedEvent("e19", mementar::Event(mementar::Fact("tape_1|isOn|Table_1", false), 134)));
  timeline->events.add(new mementar::ContextualizedEvent("e20", mementar::Event(mementar::Fact("tape_1|isBehind|grey_box", false), 134)));
  timeline->events.add(new mementar::ContextualizedEvent("e21", mementar::Event("tape_1|isIn|storage_area", 145)));
  timeline->events.add(new mementar::ContextualizedEvent("e23", mementar::Event(mementar::Fact("tape_1|isInHand|hum_green", false), 145)));
  timeline->events.add(new mementar::ContextualizedEvent("e24", mementar::Event("tape_1|isInGripper|pr2", 162)));
  timeline->events.add(new mementar::ContextualizedEvent("e25", mementar::Event(mementar::Fact("tape_1|isIn|storage_area", false), 170)));
  timeline->events.add(new mementar::ContextualizedEvent("e26", mementar::Event(mementar::Fact("tape_1|isInGripper|pr2", false), 173)));
  timeline->events.add(new mementar::ContextualizedEvent("e27", mementar::Event("tape_1|isIn|pink_box", 173,175)));

  return timeline;
}*/


// Human bleu
/*mementar::Timeline* getTimeline()
{
  mementar::Timeline* timeline = new mementar::Timeline();

  timeline->actions.add(new mementar::Action("blue_pick_1", mementar::SoftPoint(35, 36), 37));
  timeline->actions.add(new mementar::Action("blue_place_1", 37, mementar::SoftPoint(41, 42)));
  timeline->actions.add(new mementar::Action("monitor_1", 43, 51));

  timeline->events.add(new mementar::ContextualizedEvent("e0", mementar::Event("pr2|isInRoom|expe_room", 30, 31)));
  timeline->events.add(new mementar::ContextualizedEvent("e1", mementar::Event("grey_box|isOn|Table_1", 32,34)));
  timeline->events.add(new mementar::ContextualizedEvent("e2", mementar::Event("tape_1|isOn|Table_1", 32,34)));
  timeline->events.add(new mementar::ContextualizedEvent("e3", mementar::Event("tape_2|isIn|storage_area", 32,34)));
  timeline->events.add(new mementar::ContextualizedEvent("e6", mementar::Event("hum_blue|isInRoom|expe_room", 30,31)));
  timeline->events.add(new mementar::ContextualizedEvent("e7", mementar::Event("tape_1|isInHand|hum_blue", 37)));
  timeline->events.add(new mementar::ContextualizedEvent("e8", mementar::Event(mementar::Fact("tape_1|isOn|Table_1", false), 37)));
  timeline->events.add(new mementar::ContextualizedEvent("e9", mementar::Event("tape_1|isOn|Table_1", 38, 42)));
  timeline->events.add(new mementar::ContextualizedEvent("e10", mementar::Event("tape_1|isBehind|grey_box", 38)));
  timeline->events.add(new mementar::ContextualizedEvent("e11", mementar::Event(mementar::Fact("tape_1|isInHand|hum_blue", false), 42)));
  timeline->events.add(new mementar::ContextualizedEvent("e12", mementar::Event(mementar::Fact("hum_blue|isInRoom|expe_room", false), mementar::SoftPoint(51,52))));

  return timeline;
}*/

int main(int argc, char** argv)
{
  mementar::Timeline* timeline = getTimeline();

  CvFont font;
  cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, 1, 1, 0, 2);

  mementar::ActionReader actions;
  actions.read(&timeline->events, &font);

  mementar::EventReader events;
  events.read(&timeline->events, &font);

  mementar::TimelineDrawer drawer;
  drawer.draw("human_bleu.png", timeline, &actions, &events);

  /*std::cout << "width = " << ontologenius::commit_t::global_width << std::endl;
  std::cout << "height = " << ontologenius::commit_t::global_height << std::endl;

  ontologenius::TreeDrawer drawer;
  drawer.draw(getFileName(path), commit, commit_only);*/

  return 0;
}
