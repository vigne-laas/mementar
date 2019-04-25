#ifndef MEMENTAR_COMPRESSEDLEAF_H
#define MEMENTAR_COMPRESSEDLEAF_H

#include <string>
#include <vector>

#include "mementar/Btree/Btree.h"
#include "mementar/archiving_compressing/compressing/LzCompress.h"

namespace mementar
{

template<typename Tkey>
class CompressedLeaf
{
public:
  CompressedLeaf(Btree<Tkey, int>* tree, const std::string& directory);

  Tkey getKey() { return key_; }
private:
  Tkey key_;
  std::string directory_;

  std::string treeToString(Btree<Tkey, int>* tree);
};

template<typename Tkey>
CompressedLeaf<Tkey>::CompressedLeaf(Btree<Tkey, int>* tree, const std::string& directory)
{
  if(tree == nullptr)
    return;

  directory_ = directory;
  key_ = tree->getFirst()->getKey();

  LzCompress lz_comp;
  std::vector<char> out_vect;
  std::string in = treeToString(tree);
  lz_comp.compress(in, out_vect);

  lz_comp.displayCompressionRate(in.size(), out_vect.size());
  lz_comp.saveToFile(out_vect, directory_ + '/' + std::to_string(key_));
}

template<typename Tkey>
std::string CompressedLeaf<Tkey>::treeToString(Btree<Tkey, int>* tree)
{
  std::string res;
  std::vector<int> tmp_data;
  BtreeLeaf<Tkey, int>* it = tree->getFirst();
  while(it != nullptr)
  {
    tmp_data = it->getData();
    for(const auto& data : tmp_data)
      res += "[" + std::to_string(it->getKey()) + "]" + std::to_string(data) + "\n";
    it = it->next_;
  }

  return std::move(res);
}

} // namespace mementar

#endif // MEMENTAR_COMPRESSEDLEAF_H
