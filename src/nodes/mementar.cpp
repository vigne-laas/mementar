#include "ros/ros.h"

#include "mementar/archiving_compressing/archiving/Header.h"
#include "mementar/IdManager.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mementar");

  ros::NodeHandle n;

  ros::service::waitForService("ontologenius/rest", -1);

  /*std::string intern_folder = std::string(argv[1]);
  std::cout << "intern_folder " << intern_folder << std::endl;*/

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
