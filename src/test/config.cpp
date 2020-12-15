#include "mementar/core/Parametrization/Configuration.h"

#include <iostream>

int main()
{
  mementar::Configuration config;
  bool ok = true;
  ok = config.read("/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/mementar/file_intern/configuration.yaml");
  if(ok)
  {
    if(config.exist("WhiteListe"))
    {
      config["WhiteListe"].push_back("new property");
      config.display();
      ok = config.write("/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/mementar/file_intern/new_configuration.yaml");
      if(!ok)
        std::cout << "fail to write the new configuation file" << std::endl;
    }
    else
      std::cout << "WhiteListe not found" << std::endl;
  }
  else
    std::cout << "fail to read configuation file" << std::endl;
  return 0;
}
