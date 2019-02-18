#ifndef LZUNCOMPRESS_H
#define LZUNCOMPRESS_H

#include "mementar/lz/BitFileGetter.h"
#include "mementar/lz/Compressor.h"

class LzUncompress : public Compressor
{
public:
  LzUncompress();

  void uncompress(std::vector<char>& data, std::string& out);

private:
  size_t search_size_;
  size_t la_size_;

  BitFileGetter bit;

  int neededBitCount(size_t max_value);
};

#endif // LZUNCOMPRESS_H
