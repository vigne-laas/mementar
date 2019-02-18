#include "mementar/compressing/LzUncompress.h"

#include <iostream>
#include <fstream>
#include <math.h>

LzUncompress::LzUncompress() : Compressor("mlz"), bit(0, 0, 8)
{
  // la_size_ <= search_size_
  search_size_ = 0;
  la_size_ = 0;
}

void LzUncompress::uncompress(std::vector<char>& data, std::string& out)
{
  bit.set(data);

  size_t offset = 0;
  size_t length = 0;

  size_t out_file_size = 0;
  char size1 = bit.getType3();
  char size2 = bit.getType3();
  char size3 = bit.getType3();
  char size4 = bit.getType3();

  out_file_size = ((size4 << 24)&0xff000000) | ((size3 << 16)&0x00ff0000) | ((size2 << 8)&0x0000ff00) | ((size1 << 0)&0x000000ff);

  size1 = bit.getType3();
  size2 = bit.getType3();
  search_size_ = ((size2 << 8)&0x0000ff00) | ((size1 << 0)&0x000000ff);
  bit.setSize1(neededBitCount(search_size_));

  size1 = bit.getType3();
  size2 = bit.getType3();
  la_size_ = ((size2 << 8)&0x0000ff00) | ((size1 << 0)&0x000000ff);
  bit.setSize2(neededBitCount(la_size_));

  while(out.size() < out_file_size)
  {
    if(bit.getBit())
    {
      offset = bit.getType1();
      length = bit.getType2();
      out += out.substr(out.size() - offset, length);
    }
    else
      out.push_back(bit.getChar());
  }
}

int LzUncompress::neededBitCount(size_t max_value)
{
  int nb_bit = 1;
  size_t tmp_max = 2;
  while(tmp_max < max_value)
  {
    tmp_max = tmp_max<<1;
    nb_bit++;
  }
  return nb_bit;
}
