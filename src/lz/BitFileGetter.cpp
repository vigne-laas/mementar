#include "mementar/lz/BitFileGetter.h"

#include <iostream>

BitFileGetter::BitFileGetter(size_t size_1, size_t size_2, size_t size_3, size_t size_4)
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
  int16_t to_get = type_1_size_;
  for(size_t i = 0; i < type_1_size_;)
  {
    uint32_t working_data = (data_[major_index_] & 0x00ff) >> minor_index_;
    res = res | ((working_data & ~((0xffff) << to_get)) << (type_1_size_ - to_get));

    uint8_t getted = (minor_index_ + to_get >= 8) ? 8 - minor_index_ : to_get;
    i += getted;

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
    }
  }

  return res;
}

uint32_t BitFileGetter::getType2()
{
  uint32_t res = 0;
  int16_t to_get = type_2_size_;
  for(size_t i = 0; i < type_2_size_;)
  {
    uint32_t working_data = (data_[major_index_] & 0x00ff) >> minor_index_;
    res = res | ((working_data & ~((0xffff) << to_get)) << (type_2_size_ - to_get));

    uint8_t getted = (minor_index_ + to_get >= 8) ? 8 - minor_index_ : to_get;
    i += getted;

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
    }
  }

  return res;
}

uint32_t BitFileGetter::getType3()
{
  uint32_t res = 0;
  int16_t to_get = type_3_size_;
  for(size_t i = 0; i < type_3_size_;)
  {
    uint32_t working_data = (data_[major_index_] & 0x00ff) >> minor_index_;
    res = res | ((working_data & ~((0xffff) << to_get)) << (type_3_size_ - to_get));

    uint8_t getted = (minor_index_ + to_get >= 8) ? 8 - minor_index_ : to_get;
    i += getted;

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
    }
  }

  return res;
}

uint32_t BitFileGetter::getType4()
{
  uint32_t res = 0;
  int16_t to_get = type_4_size_;
  for(size_t i = 0; i < type_4_size_;)
  {
    uint32_t working_data = (data_[major_index_] & 0x00ff) >> minor_index_;
    res = res | ((working_data & ~((0xffff) << to_get)) << (type_4_size_ - to_get));

    uint8_t getted = (minor_index_ + to_get >= 8) ? 8 - minor_index_ : to_get;
    i += getted;

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
    }
  }

  return res;
}

char BitFileGetter::getChar()
{
  uint32_t res = 0;
  int16_t to_get = 7;
  for(size_t i = 0; i < 7;)
  {
    uint32_t working_data = (data_[major_index_] & 0x00ff) >> minor_index_;
    res = res | ((working_data & ~((0xffff) << to_get)) << (7 - to_get));

    uint8_t getted = (minor_index_ + to_get >= 8) ? 8 - minor_index_ : to_get;
    i += getted;

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
    }
  }

  return res;
}

bool BitFileGetter::getBit()
{
  bool res = false;

  uint32_t working_data = (data_[major_index_] & 0x00ff) >> minor_index_;
  res = working_data & 0x01;

  minor_index_ ++;
  if(minor_index_ >= 8)
  {
    minor_index_ = 0;
    major_index_++;
  }

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
