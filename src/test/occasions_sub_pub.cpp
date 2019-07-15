#include <iostream>

#include "ros/ros.h"

#include "mementar/API/TimelineManipulator.h"
#include "mementar/API/OccasionsSubscriber.h"

void callback_1(const mementar::Fact& fct)
{
  std::cout << "[CB1] " << fct() << std::endl;
}

void callback_2(const mementar::Fact& fct)
{
  std::cout << "[CB2] " << fct() << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "occasions_sub_pub");

  ros::NodeHandle n;

  mementar::TimelineManipulator manip(&n);
  manip.waitInit();

  mementar::OccasionsSubscriber sub1(&callback_1, true);
  sub1.subscribe(mementar::Fact("bob", "eat", "?"), 2);
  mementar::OccasionsSubscriber sub2(&callback_2, true);
  sub2.subscribe(mementar::Fact("max", "eat", "?"), 3);

  size_t cpt = 0;
  ros::Rate r(100);
  while((!sub1.end() || !sub2.end()) && ros::ok())
  {
    cpt++;
    if(cpt > 100)
    {
      cpt = 0;
      manip.insert(mementar::Fact("bob", "eat", "blop"));
      manip.insert(mementar::Fact("max", "eat", "blop"));
    }
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
