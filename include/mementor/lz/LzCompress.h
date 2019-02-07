#ifndef LZCOMPRESS_H
#define LZCOMPRESS_H

#include <string>
#include <vector>

#include "mementor/lz/BitFileGenerator.h"

class LzCompress
{
public:
  LzCompress(size_t search_size = 512, size_t la_size = 32);

  void compress(std::string& in, const std::string& out);

private:
  size_t search_size_;
  size_t la_size_;

  BitFileGenerator bit;

  int bitConter(size_t max_value);
};

#endif //LZCOMPRESS_H
