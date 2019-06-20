#include "ros/ros.h"

#include "mementar/RosInterface.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mementar");

  std::string directory = std::string(argv[1]);
  std::cout << "directory " << directory << std::endl;

  ros::NodeHandle n;
  mementar::RosInterface interface(&n, directory);

  interface.run();
  
  ROS_DEBUG("KILL mementar");

  return 0;
}
