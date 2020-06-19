#ifndef MEMENTAR_HUFFMAN_H
#define MEMENTAR_HUFFMAN_H

#include <array>
#include <limits>
#include <string>

#include "mementar/core/archiving_compressing/binaryManagement/BinaryManager.h"

namespace mementar
{

struct HuffCode_t
{
  uint32_t value_;
  uint8_t size_;
};

struct HuffNode_t {
  using Index = std::uint16_t;
  using Frequency = std::uint32_t;

  static constexpr Index invalid_index = std::numeric_limits<Index>::max();

  Frequency freq{};
  Index left{invalid_index};
  Index right{invalid_index};
  HuffCode_t code;
};

// Number of values representable with a byte
static constexpr HuffNode_t::Index values_in_byte = 1 << (sizeof(std::uint8_t) * 8);

// Maximum number of leaf nodes (character) in an Huffman tree
static constexpr HuffNode_t::Index leaf_count = values_in_byte;

// Maximum number of nodes (leaf + bind) in an Huffman tree
static constexpr HuffNode_t::Index node_count = leaf_count * 2;

using FrequencyMap = std::array<HuffNode_t::Frequency, leaf_count>;
using NodeList = std::array<HuffNode_t, node_count>;
using TreeList = std::array<size_t, values_in_byte>;

class Huffman : public BinaryManager
{
public:
  Huffman();
  void analyse(const std::string& data, std::size_t jobs = 1);
  void generateCode();
  std::vector<char> getTreeCode();
  std::vector<char> getDataCode(const std::string& in_data);

  size_t setTree(const std::vector<char>& in);
  std::string getFile(const std::vector<char>& data);

private:
  FrequencyMap frequencies_{};
  NodeList nodes_{};
  TreeList subtrees_{};
  size_t h_min_;

  void sum(const FrequencyMap& other, FrequencyMap& into);
  void count_char(const std::string& text, std::size_t jobs = 1);
  HuffNode_t::Index generateTree();
  void generateCode(HuffNode_t::Index index);
};

} // namespace mementar

#endif // MEMENTAR_HUFFMAN_H
