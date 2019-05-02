#ifndef MEMENTAR_COMPRESSEDLEAF_H
#define MEMENTAR_COMPRESSEDLEAF_H

#include <string>
#include <vector>

#include "mementar/Btree/Btree.h"
#include "mementar/archiving_compressing/compressing/LzCompress.h"
#include "mementar/archiving_compressing/compressing/LzUncompress.h"
#include "mementar/Fact.h"

namespace mementar
{

template<typename Tkey>
class CompressedLeaf
{
public:
  CompressedLeaf(Btree<Tkey, Fact>* tree, const std::string& directory);
  CompressedLeaf(const Tkey& key, const std::string& directory);

  std::string getDirectoty() { return directory_; }
  Tkey getKey() { return key_; }

  Btree<Tkey, Fact>* getTree();
private:
  Tkey key_;
  std::string directory_;

  std::string treeToString(Btree<Tkey, Fact>* tree);
};

template<typename Tkey>
CompressedLeaf<Tkey>::CompressedLeaf(Btree<Tkey, Fact>* tree, const std::string& directory)
{
  if(tree == nullptr)
    return;

  key_ = tree->getFirst()->getKey();
  directory_ = directory + '/' + std::to_string(key_);

  LzCompress lz_comp;
  std::vector<char> out_vect;
  std::string in = treeToString(tree);
  lz_comp.compress(in, out_vect);

  lz_comp.displayCompressionRate(in.size(), out_vect.size());
  lz_comp.saveToFile(out_vect, directory_);
}

template<typename Tkey>
CompressedLeaf<Tkey>::CompressedLeaf(const Tkey& key, const std::string& directory)
{
  key_ = key;
  size_t dot_pose = directory.find(".");
  directory_ = directory.substr(0, dot_pose);
}

template<typename Tkey>
Btree<Tkey, Fact>* CompressedLeaf<Tkey>::getTree()
{
  std::string out;

  LzUncompress lz;
  std::vector<char> data;
  if(lz.readBinaryFile(data, directory_ + ".mlz"))
  {
    lz.uncompress(data, out);
    Btree<Tkey, Fact> tree = new Btree<Tkey, Fact>();
  }

  return nullptr;
}

template<typename Tkey>
std::string CompressedLeaf<Tkey>::treeToString(Btree<Tkey, Fact>* tree)
{
  std::string res;
  std::vector<Fact> tmp_data;
  BtreeLeaf<Tkey, Fact>* it = tree->getFirst();
  while(it != nullptr)
  {
    tmp_data = it->getData();
    for(auto& data : tmp_data)
      res += "[" + std::to_string(it->getKey()) + "]" + data.toString() + "\n";
    it = it->next_;
  }

  return std::move(res);
}

} // namespace mementar

#endif // MEMENTAR_COMPRESSEDLEAF_H
