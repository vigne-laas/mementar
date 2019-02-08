#ifndef HUFFMAN_H
#define HUFFMAN_H

#include <set>
#include <string>
#include <vector>

struct HuffCode_t
{
  uint16_t value_;
  uint8_t size_;
};

struct HuffNode_t
{
  size_t freq_;
  char data_;
  HuffCode_t code_;
  HuffNode_t* left_;
  HuffNode_t* right_;

  HuffNode_t()
  {
    freq_ = 1;
    data_ = ' ';
    left_ = right_ = nullptr;
  }
  HuffNode_t(char data)
  {
    freq_ = 1;
    data_ = data;
    left_ = right_ = nullptr;
  }

  ~HuffNode_t()
  {
    if(left_ != nullptr)
      delete left_;
    if(right_ != nullptr)
      delete right_;
  }
};

class Huffman
{
public:
  void load(std::vector<char>& data);

  ~Huffman();

private:
  std::vector<HuffNode_t*> heap_;
  std::vector<char> data_;

  void generateCode(HuffNode_t* node);
};

#endif
