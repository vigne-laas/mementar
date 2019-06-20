#include "ros/ros.h"

#include "mementar/RosInterface.h"
#include "mementar/core/IdManager.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mementar");

  std::string directory = std::string(argv[1]);
  std::cout << "directory " << directory << std::endl;

  ros::NodeHandle n;
  mementar::RosInterface interface(&n, directory);

  mementar::IdManager<uint32_t> ids;

  std::cout << ids.getNewId() << std::endl; // 0
  std::cout << ids.getNewId() << std::endl; // 1
  std::cout << ids.getNewId() << std::endl; // 2
  std::cout << ids.getNewId() << std::endl;
  std::cout << ids.getNewId() << std::endl;
  std::cout << ids.getNewId() << std::endl;
  std::cout << ids.getNewId() << std::endl; // 6
  ids.removeId(2);
  std::cout << ids.getNewId() << std::endl; // 2
  std::cout << ids.getNewId() << std::endl; // 7

  ros::spin();

  ROS_DEBUG("KILL mementar");

  return 0;
}
