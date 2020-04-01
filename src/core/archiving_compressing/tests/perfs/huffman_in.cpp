#include <iostream>
#include <fstream>
#include <streambuf>

#include <chrono>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

//#include "mementar/core/archiving_compressing/compressing/Huffman.h"
#include "mementar/core/archiving_compressing/compressing/Huffman.h"

using namespace std::chrono;


/*float testHuffman(size_t nb, const std::string& input_file)
{
  std::ifstream t(input_file);
  std::string in((std::istreambuf_iterator<char>(t)),
                  std::istreambuf_iterator<char>());

  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  for(size_t i = 0; i < nb; i++)
  {
    std::vector<char> in_vect(in.begin(), in.end());

    mementar::Huffman huff;
    huff.analyse(in_vect);
    huff.generateTree();
    std::vector<char> out_vect;
    huff.getTreeCode(out_vect);
    huff.getDataCode(in_vect, out_vect);

    if(i == 0)
      huff.displayCompressionRate(in.size(), out_vect.size());
  }

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

  return time_span.count()*1000/nb; // return time span in ms
}*/

float testHuffman(size_t nb, const std::string& input_file)
{
  std::ifstream t(input_file);
  std::string in((std::istreambuf_iterator<char>(t)),
                  std::istreambuf_iterator<char>());

  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  for(size_t i = 0; i < nb; i++)
  {
    mementar::Huffman huff;
    huff.analyse(in);
    huff.generateCode();
    std::vector<char> out_vect;
    huff.getTreeCode(out_vect);
    huff.getDataCode(in, out_vect);

    if(i == 0)
    {
      huff.displayCompressionRate(in.size(), out_vect.size());
      huff.saveToFile(out_vect,  std::to_string(in.size()));
    }
  }

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

  return time_span.count()*1000/nb; // return time span in ms
}

int main (int argc, char* argv[])
{
  const size_t nb = 1;

  float time_252 = testHuffman(nb, "/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/mementar/tests_files/test_252_1MB.txt");
  float time_10 = testHuffman(nb, "/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/mementar/tests_files/test_10_9MB.txt");
  float time_6 = testHuffman(nb, "/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/mementar/tests_files/test_6_5MB.txt");

  float mean = (time_252 + time_10 + time_6) / 3.0;

  std::cout << "252 = " << time_252 << std::endl;
  std::cout << "10 = " << time_10 << std::endl;
  std::cout << "6 = " << time_6 << std::endl;
  std::cout << "mean = " << mean << std::endl;

  return 0;
}
