#include "mementor/lz/BitFileGenerator.h"

#include <iostream>

BitFileGenerator::BitFileGenerator(size_t size_1, size_t size_2, size_t size_3, size_t size_4)
{
  type_1_size_ = size_1;
  type_2_size_ = size_2;
  type_3_size_ = size_3;
  type_4_size_ = size_4;

  major_index_ = 0;
  minor_index_ = 0;

  data_.push_back(0x00);
}

void BitFileGenerator::writeType1(uint16_t value)
{
  int16_t to_add = type_1_size_;
  for(size_t i = 0; i < type_1_size_;)
  {
    data_[major_index_] = data_[major_index_] | (value << minor_index_);

    uint8_t added = 8 - minor_index_;
    i += added;

    value = value >> added;
    to_add = to_add - added;
    if(to_add >= 0)
    {
      data_.push_back(0x00);
      major_index_++;
      minor_index_ = 0;
    }
    else
      minor_index_ = 8 + to_add;
  }
}

void BitFileGenerator::writeType2(uint16_t value)
{
  int16_t to_add = type_2_size_;
  for(size_t i = 0; i < type_2_size_;)
  {
    data_[major_index_] = data_[major_index_] | (value << minor_index_);

    uint8_t added = 8 - minor_index_;
    i += added;

    value = value >> added;
    to_add = to_add - added;
    if(to_add >= 0)
    {
      data_.push_back(0x00);
      major_index_++;
      minor_index_ = 0;
    }
    else
      minor_index_ = 8 + to_add;
  }
}

void BitFileGenerator::writeType3(uint16_t value)
{
  int16_t to_add = type_3_size_;
  for(size_t i = 0; i < type_3_size_;)
  {
    data_[major_index_] = data_[major_index_] | (value << minor_index_);

    uint8_t added = 8 - minor_index_;
    i += added;

    value = value >> added;
    to_add = to_add - added;
    if(to_add >= 0)
    {
      data_.push_back(0x00);
      major_index_++;
      minor_index_ = 0;
    }
    else
      minor_index_ = 8 + to_add;
  }
}

void BitFileGenerator::writeType4(uint16_t value)
{
  int16_t to_add = type_4_size_;
  for(size_t i = 0; i < type_4_size_;)
  {
    data_[major_index_] = data_[major_index_] | (value << minor_index_);

    uint8_t added = 8 - minor_index_;
    i += added;

    value = value >> added;
    to_add = to_add - added;
    if(to_add >= 0)
    {
      data_.push_back(0x00);
      major_index_++;
      minor_index_ = 0;
    }
    else
      minor_index_ = 8 + to_add;
  }
}

void BitFileGenerator::writeChar(char value)
{
  int16_t to_add = 8;
  for(size_t i = 0; i < 8;)
  {
    data_[major_index_] = data_[major_index_] | (value << minor_index_);

    uint8_t added = 8 - minor_index_;
    i += added;

    value = value >> added;
    to_add = to_add - added;
    if(to_add >= 0)
    {
      data_.push_back(0x00);
      major_index_++;
      minor_index_ = 0;
    }
    else
      minor_index_ = 8 + to_add;
  }
}

void BitFileGenerator::writeBitTrue()
{
  data_[major_index_] = data_[major_index_] | (1 << minor_index_);
  if(minor_index_ >= 7)
  {
    data_.push_back(0x00);
    major_index_++;
    minor_index_ = 0;
  }
  else
    minor_index_++;
}

void BitFileGenerator::writeBitFalse()
{
  if(minor_index_ >= 7)
  {
    data_.push_back(0x00);
    major_index_++;
    minor_index_ = 0;
  }
  else
    minor_index_++;
}
