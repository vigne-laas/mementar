#ifndef LZCOMPRESS_H
#define LZCOMPRESS_H

#include <string>
#include <vector>

#include "mementor/lz/BitFileGenerator.h"

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
  LzCompress(size_t search_size = 512, size_t la_size = 32);

  void compress(std::string& in, const std::string& out);

  int bitConter(size_t max_value);

private:
  size_t search_size_;
  size_t la_size_;

  BitFileGenerator bit;
};

#endif //LZCOMPRESS_H
