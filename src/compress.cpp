#include <iostream>
#include <fstream>
#include <streambuf>

#include <chrono>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include "mementor/lz/LzCompress.h"

using namespace std::chrono;

int main (int argc, char* argv[])
{
  std::ifstream t("../tests_files/test.txt");
  //std::ifstream t("/home/gsarthou/Downloads/owl-export-unversioned.owl");
  std::string in((std::istreambuf_iterator<char>(t)),
                   std::istreambuf_iterator<char>());

  //std::string in = "veridique ! dominique pique nique en tunique.";

  ///////////////////////////////////////////////////////////////////
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  LzCompress comp;
	comp.compress(in, "../tests_files/out");

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  ///////////////////////////////////////////////////////////////////

  std::cout << "time = " << time_span.count() << "s" << std::endl;

  return 0;
}
