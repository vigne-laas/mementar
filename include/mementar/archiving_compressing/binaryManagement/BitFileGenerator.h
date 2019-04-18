#ifndef BITFILEGENERATOR_H
#define BITFILEGENERATOR_H

#include <vector>
#include <string>

namespace mementar
{

class BitFileGenerator
{
public:
  BitFileGenerator(uint8_t size_1 = 0, uint8_t size_2 = 0, uint8_t size_3 = 0, uint8_t size_4 = 0);

  void writeType1(uint32_t value);
  void writeType2(uint32_t value);
  void writeType3(uint32_t value);
  void writeType4(uint32_t value);
  void writeN(size_t size, uint32_t value);
  void writeChar(char value);

  void writeBitTrue();
  void writeBitFalse();

  std::vector<char> get() { return data_; }
  void get(std::vector<char>& out) { out = data_; }

private:
  std::vector<char> data_;
  size_t major_index_;
  uint8_t minor_index_;

  uint8_t type_1_size_;
  uint8_t type_2_size_;
  uint8_t type_3_size_;
  uint8_t type_4_size_;
};

} // namespace mementar

#endif // BITFILEGENERATOR_H
