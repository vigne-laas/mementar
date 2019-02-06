#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <streambuf>

#include <chrono>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

using namespace std::chrono;

struct triplet_t
{
  size_t o_;
  size_t l_;
  char c_;
  bool f_;
  triplet_t(size_t o, size_t l, char c, bool f = false)
  {
    o_ = o;
    l_ = l;
    c_ = c;
    f_ = f;
  }
};

class LzCompress
{
public:
  LzCompress()
  {
    // la_size_ <= search_size_
    search_size_ = 512;
    la_size_ = 32;
    std::cout << bitConter(la_size_) << " : " << bitConter(search_size_) << std::endl;
  }

  void compress(std::string& in, std::vector<triplet_t>& out)
  {
    size_t in_size = in.size();

    out.push_back(triplet_t(0,0,in[0]));
    size_t cursor = 1;
    size_t index = -1;

    size_t min_size = (in_size < search_size_) ? in_size : search_size_;

    while (cursor < min_size)
    {
      char c_tmp = in[cursor];
      for(index = 0; index < cursor; index++)
        if(in[index] == c_tmp)
          break;

      size_t length = 1;
      if(index == cursor)
        out.push_back(triplet_t(0,0,c_tmp));
      else
      {
        while(in[index + length] == in[cursor + length])
          length++;

        if(length > 1)
          out.push_back(triplet_t(cursor - index, length, c_tmp, true));
        else
          out.push_back(triplet_t(0, 0, c_tmp));
      }

      cursor += length;
    }

    while (cursor < in_size)
    {
      char c_tmp = in[cursor];
      for(index = cursor - search_size_; index < cursor; index++)
        if(in[index] == c_tmp)
          break;

      size_t length = 1;
      if(index == cursor)
        out.push_back(triplet_t(0,0,c_tmp));
      else
      {
        while(in[index + length] == in[cursor + length])
          length++;

        if(length > 1)
          out.push_back(triplet_t(cursor - index, length, c_tmp, true));
        else
          out.push_back(triplet_t(0, 0, c_tmp));
      }

      cursor += length;
    }
  }

  int bitConter(size_t max_value)
  {
    int nb_bit = 1;
    int tmp_max = 2;
    while(tmp_max < max_value)
    {
      tmp_max = tmp_max<<1;
      nb_bit++;
    }
    return nb_bit;
  }
private:
  size_t search_size_;
  size_t la_size_;
};

int main (int argc, char* argv[])
{
  LzCompress comp;
  std::vector<triplet_t> out;

  //std::ifstream t("../syslog");
  std::ifstream t("/home/gsarthou/Downloads/owl-export-unversioned.owl");
  std::string in((std::istreambuf_iterator<char>(t)),
                   std::istreambuf_iterator<char>());

  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  comp.compress(in, out);
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

  std::cout << "time = " << time_span.count() << "s" << std::endl;

  /*for(auto trip : out)
  {
    if(trip.f_)
      std::cout << trip.o_ << " : " << trip.l_ << std::endl;
    else
      std::cout << trip.c_ << std::endl;
  }*/

  size_t size_out_base = 0;
  size_t size_out_comp = 0;
  for(auto trip : out)
  if(trip.f_)
    size_out_comp += 15;
  else
    size_out_base += 9;

  std::cout << out.size() << " / " << in.size() << " = " << 1 - (out.size() / (float)in.size()) << std::endl;
  std::cout << size_out_base + size_out_comp << " / " << in.size()*8 << " = " << 1 - ((size_out_base + size_out_comp) / (float)(in.size()*8)) << std::endl;

  return 0;
}
