#include "ros/ros.h"

#include "mementar/core/Parametrization/Parameters.h"
#include "mementar/RosInterface.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mementar");

  mementar::Parameters params;
  params.insert(mementar::Parameter("directory", {"-d", "--directory"}, {"none"}));
  params.insert(mementar::Parameter("config", {"-c", "--config"}, {"none"}));

  params.set(argc, argv);
  params.display();

  ros::NodeHandle n("mementar");
  mementar::RosInterface interface(&n,
                                   params.parameters_.at("directory").getFirst(),
                                   params.parameters_.at("config").getFirst());

  interface.run();

  ROS_DEBUG("KILL mementar");

  return 0;
}
