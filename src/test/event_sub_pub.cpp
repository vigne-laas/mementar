#include <iostream>

#include "ros/ros.h"

#include "mementar/API/TimelineManipulator.h"
#include "mementar/API/EventsSubscriber.h"

void callback_1(const mementar::Event& evt)
{
  std::cout << "[CB1] " << evt() << std::endl;
}

void callback_2(const mementar::Event& evt)
{
  std::cout << "[CB2] " << evt() << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "event_sub_pub");

  ros::NodeHandle n;

  mementar::TimelineManipulator manip(&n);
  manip.waitInit();

  mementar::EventsSubscriber sub1(&callback_1, true);
  sub1.subscribe(mementar::Event("bob", "eat", "?"), 2);
  mementar::EventsSubscriber sub2(&callback_2, true);
  sub2.subscribe(mementar::Event("max", "eat", "?"), 3);

  size_t cpt = 0;
  ros::Rate r(100);
  while((!sub1.end() || !sub2.end()) && ros::ok())
  {
    cpt++;
    if(cpt > 100)
    {
      cpt = 0;
      manip.insert(mementar::Event("bob", "eat", "blop"));
      manip.insert(mementar::Event("max", "eat", "blop"));
    }
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
