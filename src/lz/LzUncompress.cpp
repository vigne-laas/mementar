#include "mementor/lz/LzUncompress.h"

#include <iostream>
#include <fstream>

LzUncompress::LzUncompress(size_t search_size, size_t la_size) : bit(bitConter(search_size), bitConter(la_size))
{
  // la_size_ <= search_size_
  search_size_ = search_size;
  la_size_ = la_size;
}

void LzUncompress::uncompress(const std::string& in, std::string& out)
{
	std::ifstream infile(in, std::ios::binary | std::ios::ate);
  std::streamsize size = infile.tellg();
  infile.seekg(0, std::ios::beg);

  std::vector<char> buffer(size);
  if(infile.read(buffer.data(), size))
  {
    bit.set(buffer);

    size_t found_size = bitConter(search_size_) + bitConter(la_size_);
    size_t not_found_size = 8;

    bool found = false;
    size_t offset = 0;
    size_t length = 0;

    size_t out_file_size = 0;
    char size1 = bit.getChar();
    char size2 = bit.getChar();
    char size3 = bit.getChar();
    char size4 = bit.getChar();

    out_file_size = ((size4 << 24)&0xff000000) | ((size3 << 16)&0x00ff0000) | ((size2 << 8)&0x0000ff00) | ((size1 << 0)&0x000000ff);

    bool end = false;
    while(out.size() < out_file_size)
    {
      found = bit.getBit();
      if(found)
      {
        offset = bit.getType1();
        length = bit.getType2();
        out += out.substr(out.size() - offset, length);
      }
      else
        out.push_back(bit.getChar());
    }
  }
}

int LzUncompress::bitConter(size_t max_value)
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
