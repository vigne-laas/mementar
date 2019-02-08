#ifndef BITFILEGENERATOR_H
#define BITFILEGENERATOR_H

#include <vector>
#include <string>

class BitFileGenerator
{
public:
  BitFileGenerator(size_t size_1 = 0, size_t size_2 = 0, size_t size_3 = 0, size_t size_4 = 0);

  void writeType1(uint16_t value);
  void writeType2(uint16_t value);
  void writeType3(uint16_t value);
  void writeType4(uint16_t value);
  void writeN(size_t size, uint16_t value);
  void writeChar(char value);

  void writeBitTrue();
  void writeBitFalse();

  std::vector<char> get() { return data_; }

private:
  std::vector<char> data_;
  size_t major_index_;
  size_t minor_index_;

  size_t type_1_size_;
  size_t type_2_size_;
  size_t type_3_size_;
  size_t type_4_size_;
};

#endif // BITFILEGENERATOR_H
