#include "mementar/lz/Huffman.h"

#include <iostream>
#include <algorithm>

#include "mementar/lz/BitFileGenerator.h"

bool comparePtrNode(HuffNode_t* a, HuffNode_t* b)
{
  return (a->freq_ < b->freq_);
}

Huffman::~Huffman()
{
  for(auto it : heap_)
    delete it;
}

void Huffman::analyse(std::vector<char>& data)
{
  for(const auto& c : data)
  {
    auto it = leaf_map_.find(c);
    if(it == leaf_map_.end())
    {
      HuffNode_t* leaf = new HuffNode_t(c);
      heap_.push_back(leaf);
      leaf_map_[c] = leaf;
    }
    else
      it->second->freq_++;
  }
}

void Huffman::generateTree()
{
  while(heap_.size() > 1)
  {
    std::sort(heap_.begin(), heap_.end(), comparePtrNode);

    HuffNode_t* tmp = new HuffNode_t();
    tmp->right_ = heap_[0];
    tmp->left_ = heap_[1];
    tmp->freq_ = tmp->right_->freq_ + tmp->left_->freq_;
    heap_.erase(heap_.begin());
    heap_.erase(heap_.begin());
    heap_.push_back(tmp);
  }

  heap_[0]->code_.value_ = 0;
  heap_[0]->code_.size_ = 0;
  generateCode(heap_[0]);
}

void Huffman::generateCode(HuffNode_t* node)
{
  if(node->right_ != nullptr)
  {
    node->right_->code_.value_ = node->code_.value_ << 1;
    node->right_->code_.size_ = node->code_.size_ + 1;
    generateCode(node->right_);
  }
  if(node->left_ != nullptr)
  {
    node->left_->code_.value_ = (node->code_.value_ << 1) | 0x01;
    node->left_->code_.size_ = node->code_.size_ + 1;
    generateCode(node->left_);
  }
}

void Huffman::getTreeCode(std::vector<char>& out)
{
  BitFileGenerator bit(8, 4, 13);
  // //coding tree
  bit.writeType1(leaf_map_.size());
  for(auto it : leaf_map_)
  {
    bit.writeType1(it.second->data_);
    bit.writeType2(it.second->code_.size_);
    bit.writeType3(it.second->code_.value_);
  }
  std::vector<char> tmp = bit.get();
  out.insert(out.begin(), tmp.begin(), tmp.end());
}

void Huffman::getDataCode(std::vector<char>& data, std::vector<char>& out)
{
  BitFileGenerator bit;

  for(const auto& c : data)
  {
    auto it = leaf_map_.find(c);
    bit.writeN(it->second->code_.size_, it->second->code_.value_);
  }
  std::vector<char> tmp = bit.get();
  out.insert(out.begin(), tmp.begin(), tmp.end());
}
