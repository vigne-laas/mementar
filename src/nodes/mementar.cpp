#include "ros/ros.h"

#include "mementar/archiving_compressing/archiving/Header.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mementar");

  ros::NodeHandle n;

  ros::service::waitForService("ontologenius/rest", -1);

  std::string intern_folder = std::string(argv[1]);
  std::cout << "intern_folder " << intern_folder << std::endl;

  ros::spin();

  ROS_DEBUG("KILL mementar");

  return 0;
}
