#ifndef LZUNCOMPRESS_H
#define LZUNCOMPRESS_H

#include "mementar/lz/BitFileGetter.h"

class LzUncompress
{
public:
  LzUncompress(size_t search_size = 1024, size_t la_size = 64);

  void uncompress(const std::string& in, std::string& out);

private:
  size_t search_size_;
  size_t la_size_;

  BitFileGetter bit;

  int bitConter(size_t max_value);
};

#endif // LZUNCOMPRESS_H
