#include "mementar/lz/LzCompress.h"

#include <iostream>
#include <fstream>

LzCompress::LzCompress(size_t search_size, size_t la_size) : bit(neededBitCount(search_size), neededBitCount(la_size), 8)
{
  // la_size_ <= search_size_
  search_size_ = search_size;
  search_size_1_ = search_size - 1;
  la_size_ = la_size;
  la_size_1_ = la_size_ - 1;
}

void LzCompress::compress(std::string& in, const std::string& out)
{
  size_t in_size = in.size();

  bit.writeType3(in_size >> 0);
  bit.writeType3(in_size >> 8);
  bit.writeType3(in_size >> 16);
  bit.writeType3(in_size >> 24);

  bit.writeType3(search_size_ >> 0);
  bit.writeType3(search_size_ >> 8);

  bit.writeType3(la_size_ >> 0);
  bit.writeType3(la_size_ >> 8);

  bit.writeBitFalse();
  bit.writeChar(in[0]);

  size_t cursor = 1;
  size_t index = -1;
  size_t length = 1;
  size_t tmp_length = 1;
  size_t search_index = 1;
  size_t la_index = 1;
  size_t i = -1;

  size_t min_size = (in_size < search_size_) ? in_size : search_size_;

  while (cursor < min_size)
  {
    length = 1;
    char c_tmp = in[cursor];
    for(i = 0; i < cursor; i++)
    {
      if(in[i] == c_tmp)
      {
        tmp_length = 1;
        search_index = i + 1;
        la_index = cursor + 1;
        while((in[search_index] == in[la_index]) && (tmp_length < la_size_1_) && (search_index < cursor))
        {
          tmp_length++;
          search_index++;
          la_index++;
        }

        if(tmp_length > length)
        {
          length = tmp_length;
          index = i;
        }
      }
    }

    if(length > 1)
    {
      bit.writeBitTrue();
      bit.writeType1(cursor - index);
      bit.writeType2(length);
    }
    else
    {
      bit.writeBitFalse();
      bit.writeChar(c_tmp);
    }

    cursor += length;
  }

  while (cursor < in_size)
  {
    length = 1;
    char c_tmp = in[cursor];
    for(i = cursor - search_size_1_; i < cursor; i++)
    {
      if(in[i] == c_tmp)
      {
        tmp_length = 1;
        search_index = i + 1;
        la_index = cursor + 1;
        while((in[search_index] == in[la_index]) && (tmp_length < la_size_1_) && (search_index < cursor))
        {
          tmp_length++;
          search_index++;
          la_index++;
        }

        if(tmp_length > length)
        {
          length = tmp_length;
          index = i;
        }
      }
    }

    if(length > 1)
    {
      bit.writeBitTrue();
      bit.writeType1(cursor - index);
      bit.writeType2(length);
    }
    else
    {
      bit.writeBitFalse();
      bit.writeChar(c_tmp);
    }

    cursor += length;
  }

  std::vector<char> out_vect = bit.get();

  std::cout << "Compression rate : " << (1 - (out_vect.size() / (float)in.size())) * 100.0f << std::endl;

  std::ofstream outfile;
	outfile.open(out + ".mlz", std::ios::binary | std::ios::out);
  std::string str(out_vect.begin(), out_vect.end());
	outfile.write(str.c_str(), str.length());
	outfile.close();

  std::cout << "Saved into " << out << ".mlz" << std::endl;
}

int LzCompress::neededBitCount(size_t max_value)
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
