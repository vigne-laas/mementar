#include "mementar/lz/Huffman.h"

#include <map>
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

void Huffman::load(std::vector<char>& data)
{
  std::map<char, HuffNode_t*> tmp_map;
  for(const auto& c : data)
  {
    auto it = tmp_map.find(c);
    if(it == tmp_map.end())
    {
      HuffNode_t* leaf = new HuffNode_t(c);
      heap_.push_back(leaf);
      tmp_map[c] = leaf;
    }
    else
      it->second->freq_++;
  }

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

  BitFileGenerator bit(8, 8, 16);

  for(auto it : tmp_map)
  {
    bit.writeType1(it.second->data_);
    bit.writeType1(it.second->code_.size_);
    bit.writeType1(it.second->code_.value_);
  }

  for(const auto& c : data)
  {
    auto it = tmp_map.find(c);
    bit.writeN(it->second->code_.size_, it->second->code_.value_);
  }
  data = bit.get();
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
