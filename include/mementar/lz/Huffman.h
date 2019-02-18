#ifndef HUFFMAN_H
#define HUFFMAN_H

#include <set>
#include <string>
#include <vector>
#include <map>
#include "mementar/lz/Compressor.h"

struct HuffCode_t
{
  uint32_t value_;
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

class Huffman : public Compressor
{
public:
  void analyse(std::vector<char>& data);
  void generateTree();
  void getTreeCode(std::vector<char>& out);
  void getDataCode(std::vector<char>& data, std::vector<char>& out);

  size_t setTree(std::vector<char>& in);
  void getFile(std::vector<char>& data, std::string& out);

  Huffman();
  ~Huffman();

private:
  std::vector<HuffNode_t*> heap_;
  std::map<char, HuffNode_t*> leaf_map_;

  void generateCode(HuffNode_t* node);
};

#endif
