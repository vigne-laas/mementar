#include "mementar/lz/BitFileGetter.h"

#include <iostream>

BitFileGetter::BitFileGetter(uint32_t size_1, uint32_t size_2, uint32_t size_3, uint32_t size_4)
{
  type_1_size_ = size_1;
  type_2_size_ = size_2;
  type_3_size_ = size_3;
  type_4_size_ = size_4;

  major_index_ = 0;
  minor_index_ = 0;
}

uint32_t BitFileGetter::getType1()
{
  uint32_t res = 0;
  int8_t to_get = type_1_size_;
  while(true)
  {
    uint32_t working_data = (data_[major_index_] & 0x000000ff) >> minor_index_;
    res |= ((working_data & ~((0xffffffff) << to_get)) << (type_1_size_ - to_get));

    uint8_t getted = (minor_index_ + to_get >= 8) ? 8 - minor_index_ : to_get;

    to_get -= getted;
    if(to_get > 0)
    {
      major_index_++;
      minor_index_ = 0;
    }
    else
    {
      minor_index_ += getted;
      if(minor_index_ >= 8)
      {
        minor_index_ = 0;
        major_index_++;
      }
      break;
    }
  }

  return res;
}

uint32_t BitFileGetter::getType2()
{
  uint32_t res = 0;
  int8_t to_get = type_2_size_;
  while(true)
  {
    uint32_t working_data = (data_[major_index_] & 0x000000ff) >> minor_index_;
    res |= ((working_data & ~((0xffffffff) << to_get)) << (type_2_size_ - to_get));

    uint8_t getted = (minor_index_ + to_get >= 8) ? 8 - minor_index_ : to_get;

    to_get -= getted;
    if(to_get > 0)
    {
      major_index_++;
      minor_index_ = 0;
    }
    else
    {
      minor_index_ += getted;
      if(minor_index_ >= 8)
      {
        minor_index_ = 0;
        major_index_++;
      }
      break;
    }
  }

  return res;
}

uint32_t BitFileGetter::getType3()
{
  uint32_t res = 0;
  int8_t to_get = type_3_size_;
  while(true)
  {
    uint32_t working_data = (data_[major_index_] & 0x000000ff) >> minor_index_;
    res |= ((working_data & ~((0xffffffff) << to_get)) << (type_3_size_ - to_get));

    uint8_t getted = (minor_index_ + to_get >= 8) ? 8 - minor_index_ : to_get;

    to_get -= getted;
    if(to_get > 0)
    {
      major_index_++;
      minor_index_ = 0;
    }
    else
    {
      minor_index_ += getted;
      if(minor_index_ >= 8)
      {
        minor_index_ = 0;
        major_index_++;
      }
      break;
    }
  }

  return res;
}

uint32_t BitFileGetter::getType4()
{
  uint32_t res = 0;
  int8_t to_get = type_4_size_;
  while(true)
  {
    uint32_t working_data = (data_[major_index_] & 0x000000ff) >> minor_index_;
    res |= ((working_data & ~((0xffffffff) << to_get)) << (type_4_size_ - to_get));

    uint8_t getted = (minor_index_ + to_get >= 8) ? 8 - minor_index_ : to_get;

    to_get -= getted;
    if(to_get > 0)
    {
      major_index_++;
      minor_index_ = 0;
    }
    else
    {
      minor_index_ += getted;
      if(minor_index_ >= 8)
      {
        minor_index_ = 0;
        major_index_++;
      }
      break;
    }
  }

  return res;
}

char BitFileGetter::getChar()
{
  char res = 0;
  int8_t to_get = 7;
  while(true)
  {
    uint32_t working_data = (data_[major_index_] & 0x000000ff) >> minor_index_;
    res |= ((working_data & ~((0xffffffff) << to_get)) << (7 - to_get));

    uint8_t getted = (minor_index_ + to_get >= 8) ? 8 - minor_index_ : to_get;

    to_get -= getted;
    if(to_get > 0)
    {
      major_index_++;
      minor_index_ = 0;
    }
    else
    {
      minor_index_ += getted;
      if(minor_index_ >= 8)
      {
        minor_index_ = 0;
        major_index_++;
      }
      break;
    }
  }

  return res;
}

bool BitFileGetter::getBit()
{
  bool res = (data_[major_index_] >> minor_index_) & 0x01;

  if(minor_index_ >= 7)
  {
    minor_index_ = 0;
    major_index_++;
  }
  else
    minor_index_ ++;

  return res;
}

bool BitFileGetter::end(size_t offset)
{
  if(major_index_ >= data_.size())
  {
    if(minor_index_ + offset >= 8)
      return true;
    else
      return false;
  }
  else
    return false;
}
