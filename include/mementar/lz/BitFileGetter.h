#ifndef BITFILEGETTER_H
#define BITFILEGETTER_H

#include <vector>
#include <string>

class BitFileGetter
{
public:
  BitFileGetter(size_t size_1 = 0, size_t size_2 = 0, size_t size_3 = 0, size_t size_4 = 0);

  void setSize1(size_t size_1) { type_1_size_ = size_1; }
  void setSize2(size_t size_2) { type_2_size_ = size_2; }
  void setSize3(size_t size_3) { type_3_size_ = size_3; }
  void setSize4(size_t size_4) { type_4_size_ = size_4; }

  uint16_t getType1();
  uint16_t getType2();
  uint16_t getType3();
  uint16_t getType4();
  char getChar();

  bool getBit();

  bool end(size_t offset = 0);

  void set(std::vector<char>& data) { data_ = data; }

private:
  std::vector<char> data_;
  size_t major_index_;
  size_t minor_index_;

  size_t type_1_size_;
  size_t type_2_size_;
  size_t type_3_size_;
  size_t type_4_size_;
};

#endif // BITFILEGETTER_H
