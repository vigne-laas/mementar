#include "mementar/archiving_compressing/binaryManagement/BitFileGenerator.h"

namespace mementar
{

BitFileGenerator::BitFileGenerator(uint8_t size_1, uint8_t size_2, uint8_t size_3, uint8_t size_4)
{
  type_1_size_ = size_1;
  type_2_size_ = size_2;
  type_3_size_ = size_3;
  type_4_size_ = size_4;

  major_index_ = 0;
  minor_index_ = 0;

  data_.push_back(0x00);
}

void BitFileGenerator::writeType1(uint32_t value)
{
  int8_t to_add = type_1_size_;
  do
  {
    data_[major_index_] |= ((value & ~(0xffffffff << to_add)) << minor_index_);

    uint8_t added = 8 - minor_index_;

    value >>= added;
    to_add -= added;
    if(to_add >= 0)
    {
      data_.push_back(0x00);
      major_index_++;
      minor_index_ = 0;
    }
    else
      minor_index_ = 8 + to_add;
  }
  while(to_add > 0);
}

void BitFileGenerator::writeType2(uint32_t value)
{
  int8_t to_add = type_2_size_;
  do
  {
    data_[major_index_] |= ((value & ~(0xffffffff << to_add)) << minor_index_);

    uint8_t added = 8 - minor_index_;

    value >>= added;
    to_add -= added;
    if(to_add >= 0)
    {
      data_.push_back(0x00);
      major_index_++;
      minor_index_ = 0;
    }
    else
      minor_index_ = 8 + to_add;
  }
  while(to_add > 0);
}

void BitFileGenerator::writeType3(uint32_t value)
{
  int8_t to_add = type_3_size_;
  do
  {
    data_[major_index_] |= ((value & ~(0xffffffff << to_add)) << minor_index_);

    uint8_t added = 8 - minor_index_;

    value >>= added;
    to_add -= added;
    if(to_add >= 0)
    {
      data_.push_back(0x00);
      major_index_++;
      minor_index_ = 0;
    }
    else
      minor_index_ = 8 + to_add;
  }
  while(to_add > 0);
}

void BitFileGenerator::writeType4(uint32_t value)
{
  int8_t to_add = type_4_size_;
  do
  {
    data_[major_index_] |= ((value & ~(0xffffffff << to_add)) << minor_index_);

    uint8_t added = 8 - minor_index_;

    value >>= added;
    to_add -= added;
    if(to_add >= 0)
    {
      data_.push_back(0x00);
      major_index_++;
      minor_index_ = 0;
    }
    else
      minor_index_ = 8 + to_add;
  }
  while(to_add > 0);
}

void BitFileGenerator::writeN(size_t size, uint32_t value)
{
  int8_t to_add = size;
  do
  {
    data_[major_index_] |= ((value & ~(0xffffffff << to_add)) << minor_index_);

    uint8_t added = 8 - minor_index_;

    value >>= added;
    to_add -= added;
    if(to_add >= 0)
    {
      data_.push_back(0x00);
      major_index_++;
      minor_index_ = 0;
    }
    else
      minor_index_ = 8 + to_add;
  }
  while(to_add > 0);
}

void BitFileGenerator::writeChar(char value)
{
  int8_t to_add = 7;
  do
  {
    data_[major_index_] |= ((value & ~(0xffffffff << to_add)) << minor_index_);

    uint8_t added = 8 - minor_index_;

    value >>= added;
    to_add -= added;
    if(to_add >= 0)
    {
      data_.push_back(0x00);
      major_index_++;
      minor_index_ = 0;
    }
    else
      minor_index_ = 8 + to_add;
  }
  while(to_add > 0);
}

void BitFileGenerator::writeBitTrue()
{
  data_[major_index_] |= (1 << minor_index_);
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

} // namespace mementar
