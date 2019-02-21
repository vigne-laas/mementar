#include "mementar/archiving_compressing/compressing/LzUncompress.h"

#include <iostream>
#include <fstream>
#include <math.h>

LzUncompress::LzUncompress() : BinaryManager("mlz"), bit(0, 0, 8)
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

  auto out_file_size = toInteger<size_t>({(uint8_t)bit.getType3(), (uint8_t)bit.getType3(), (uint8_t)bit.getType3(), (uint8_t)bit.getType3()});

  search_size_ = toInteger<uint16_t>({(uint8_t)bit.getType3(), (uint8_t)bit.getType3()});
  bit.setSize1(neededBitCount(search_size_));

  la_size_ = toInteger<uint16_t>({(uint8_t)bit.getType3(), (uint8_t)bit.getType3()});
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
