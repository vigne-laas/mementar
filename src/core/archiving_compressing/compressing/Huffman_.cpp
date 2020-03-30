#include "mementar/core/archiving_compressing/compressing/Huffman_.h"

#include <algorithm>
#include <future>
#include <queue>
#include <thread>
#include <vector>

#include "mementar/core/archiving_compressing/binaryManagement/BitFileGenerator.h"

#define TREE_CHAR_SIZE 8
#define TREE_VALUE_SIZE 6
#define TREE_VALUE_SIZE_SIZE 31 //do not go over 31

template <typename T, std::size_t N>
struct static_alloc {
    using value_type = T;
    static_alloc() = default;
    template <typename U>
    constexpr static_alloc(const static_alloc<U, N>&) noexcept {};
    [[nodiscard]] constexpr value_type* allocate(std::size_t n) {
        return buffer.data();
    }
    constexpr void deallocate(value_type* p, std::size_t n) {}
    template <typename U>
    struct rebind {
        using other = static_alloc<U, N>;
    };
    std::array<value_type, N> buffer{};
};
template <typename T, typename U, std::size_t N>
bool operator==(const static_alloc<T, N>&, const static_alloc<U, N>&) {
    return true;
}
template <typename T, typename U, std::size_t N>
bool operator!=(const static_alloc<T, N>&, const static_alloc<U, N>&) {
    return false;
}

namespace mementar {

  struct NodeCompare
  {
  public:
    explicit NodeCompare(const NodeList& nodes) : nodes_{nodes} {}
    bool operator()(HuffNode_t::Index l, HuffNode_t::Index r)
    {
      return nodes_[l].freq > nodes_[r].freq;
    }

  private:
    const NodeList& nodes_;
  };

  Huffman_::Huffman_() : BinaryManager("mhu")
  {

  }

  void Huffman_::generateTree(const std::string& data, std::size_t jobs)
  {
    auto freqs = count_char(data, jobs);
    auto root_node = generateTree(freqs);
    nodes_[root_node].code.size_ = 0;
    nodes_[root_node].code.value_ = 0;
    generateCode(root_node);
  }

  void Huffman_::getTreeCode(std::vector<char>& out)
  {
    BitFileGenerator bit(TREE_CHAR_SIZE, TREE_VALUE_SIZE, TREE_VALUE_SIZE_SIZE);
    size_t valid_leaf = 0;
    for(size_t i = 0; i < leaf_count; i++)
      if(nodes_[i].freq != 0)
        valid_leaf++;

    //coding tree
    bit.writeType1((valid_leaf >> 0) & 0x000000ff);
    bit.writeType1((valid_leaf >> 8) & 0x000000ff);

    for(size_t i = 0; i < leaf_count; i++)
    {
      if(nodes_[i].freq != 0)
      {
        bit.writeType1((char)i);
        bit.writeType2(nodes_[i].code.size_);
        bit.writeType3(nodes_[i].code.value_);
      }
    }
    std::vector<char> tmp;
    bit.get(tmp);
    out.insert(out.end(), tmp.begin(), tmp.end());
  }

  void Huffman_::getDataCode(const std::vector<char>& data, std::vector<char>& out)
  {
    BitFileGenerator bit(8);
    bit.resize(data.size());

    bit.writeType1((data.size() >> 0) & 0x000000ff);
    bit.writeType1((data.size() >> 8) & 0x000000ff);
    bit.writeType1((data.size() >> 16) & 0x000000ff);
    bit.writeType1((data.size() >> 24) & 0x000000ff);

    for(const std::uint8_t& c : data)
      bit.writeNReverse(nodes_[c].code.size_, nodes_[c].code.value_);
    
    std::vector<char> tmp = bit.get();
    out.insert(out.end(), tmp.begin(), tmp.end());
  }

  void Huffman_::sum(const FrequencyMap& other, FrequencyMap& into)
  {
    std::transform(std::begin(other), std::end(other), std::begin(into),
                   std::begin(into), std::plus<>{});
  }

  FrequencyMap count_char_impl(const std::string& text)
  {
    FrequencyMap freq{};  // Zero-initialized array
    for (const std::uint8_t& c : text)
      freq[c]++;
    return freq;
  }

  FrequencyMap Huffman_::count_char(const std::string& text, std::size_t jobs)
  {
    if (jobs <= 1)
      return count_char_impl(text);

    const auto thread_count = jobs - 1;
    const auto bound = text.size() / thread_count;

    std::vector<std::future<FrequencyMap>> counting_units(thread_count);

    size_t lower_bound = 0;
    for (auto& unit : counting_units)
    {
      unit = std::async(std::launch::async, count_char_impl,
                        text.substr(lower_bound, bound));
      lower_bound += bound;
    }
    auto result = count_char_impl(text.substr(lower_bound));

    for (auto& unit : counting_units)
      sum(unit.get(), result);

    return result;
  }

  HuffNode_t::Index Huffman_::generateTree(const FrequencyMap& freq)
  {
    using minheap = std::priority_queue<
                    HuffNode_t::Index,
                    std::vector<HuffNode_t::Index, static_alloc<HuffNode_t::Index, leaf_count>>,
                    NodeCompare>;

    minheap heap{NodeCompare{nodes_}};

    for (HuffNode_t::Index i = 0; i < freq.size(); ++i)
    {
      if (freq[i] == 0)
        continue;  // Do not set unused characters
      nodes_[i].freq = freq[i];
      heap.push(i);
    }

    HuffNode_t::Index bind_node_index = leaf_count;  // bind nodes are stored after all leaves

    while (heap.size() != 1)
    {
      auto right = heap.top();
      heap.pop();
      auto left = heap.top();
      heap.pop();

      nodes_[bind_node_index].freq =
      nodes_[left].freq + nodes_[right].freq;
      nodes_[bind_node_index].left = left;
      nodes_[bind_node_index].right = right;
      heap.push(bind_node_index);

      ++bind_node_index;
    }
    return heap.top();
  }

  void Huffman_::generateCode(HuffNode_t::Index index)
  {
    if(nodes_[index].right != HuffNode_t::invalid_index)
    {
      nodes_[nodes_[index].right].code.value_ = (nodes_[index].code.value_ << 1);
      nodes_[nodes_[index].right].code.size_ = nodes_[index].code.size_ + 1;
      generateCode(nodes_[index].right);
    }
    if(nodes_[index].left != HuffNode_t::invalid_index)
    {
      nodes_[nodes_[index].left].code.value_ = (nodes_[index].code.value_ << 1) | 0x01;
      nodes_[nodes_[index].left].code.size_ = nodes_[index].code.size_ + 1;
      generateCode(nodes_[index].left);
    }
  }
}
