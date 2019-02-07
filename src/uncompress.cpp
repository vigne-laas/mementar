#include <iostream>
#include <fstream>
#include <streambuf>

#include <chrono>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include "mementor/lz/BitFileGetter.h"
#include "mementor/lz/LzUncompress.h"

using namespace std::chrono;

int main (int argc, char* argv[])
{
  std::string input_file = "";
  std::string output_file = "";

  /*if (argc != 5)
  {
    std::cout << "not enough arguments " << argc << std::endl;
    return -1;
  }

  if(std::string(argv[1]) == "-i")
    input_file = std::string(argv[2]);
  else if(std::string(argv[1]) == "-o")
    output_file = std::string(argv[2]);
  else
  {
    std::cout << "arguments must be -i or -o " << std::string(argv[1]) << std::endl;
    return -1;
  }

  if(std::string(argv[3]) == "-i")
    input_file = std::string(argv[4]);
  else if(std::string(argv[3]) == "-o")
    output_file = std::string(argv[4]);
  else
  {
    std::cout << "arguments must be -i or -o" << std::string(argv[4]) << std::endl;
    return -1;
  }*/

  std::string out;

  //std::ifstream t(input_file);
  //std::ifstream t("../tests_files/syslog");
  //std::ifstream t("/home/gsarthou/Downloads/owl-export-unversioned.owl");
  /*std::string in((std::istreambuf_iterator<char>(t)),
                   std::istreambuf_iterator<char>());*/

  ///////////////////////////////////////////////////////////////////
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  LzUncompress lz;
  lz.uncompress("../tests_files/syslog.mlz", out);

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  ///////////////////////////////////////////////////////////////////

  //std::cout << out << std::endl;

  std::cout << "time = " << time_span.count() << "s" << std::endl;

  return 0;
}
