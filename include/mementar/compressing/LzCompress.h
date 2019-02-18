#ifndef LZCOMPRESS_H
#define LZCOMPRESS_H

#include <string>
#include <vector>

#include "mementar/compressing/BitFileGenerator.h"
#include "mementar/compressing/Compressor.h"

class LzCompress : public Compressor
{
public:
  LzCompress(size_t search_size = 2048, size_t la_size = 64);

  void compress(std::string& in, std::vector<char>& out);

private:
  size_t search_size_;
  size_t search_size_1_;
  size_t la_size_;
  size_t la_size_1_;

  BitFileGenerator bit;

  int neededBitCount(size_t max_value);
};

#endif //LZCOMPRESS_H
