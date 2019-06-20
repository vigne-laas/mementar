#include <vector>
#include <regex>

#include "mementar/core/EpisodicTree/CompressedLeaf.h"

#include "mementar/core/archiving_compressing/compressing/LzCompress.h"
#include "mementar/core/archiving_compressing/compressing/LzUncompress.h"

namespace mementar
{

CompressedLeaf::CompressedLeaf(Btree<time_t, Fact>* tree, const std::string& directory)
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

CompressedLeaf::CompressedLeaf(const time_t& key, const std::string& directory)
{
  key_ = key;
  size_t dot_pose = directory.find(".");
  directory_ = directory.substr(0, dot_pose);
}

Btree<time_t, Fact>* CompressedLeaf::getTree()
{
  std::string out;

  LzUncompress lz;
  std::vector<char> data;
  if(lz.readBinaryFile(data, directory_ + ".mlz"))
  {
    lz.uncompress(data, out);
    Btree<time_t, Fact>* tree = new Btree<time_t, Fact>();

    std::regex regex("\\[(\\d+)\\](\\w+)\\|(\\w+)\\|(\\w+)");
    std::smatch match;

    std::istringstream iss(out);
    std::string line;
    while(std::getline(iss, line))
    {
      if(std::regex_match(line, match, regex))
      {
        Fact fact(match[2].str(), match[3].str(), match[4].str());
        time_t key;
        std::istringstream iss(match[1].str());
        iss >> key;

        tree->insert(key, fact);
      }
    }

    return tree;
  }

  return nullptr;
}

std::string CompressedLeaf::treeToString(Btree<time_t, Fact>* tree)
{
  std::string res;
  std::vector<Fact> tmp_data;
  BtreeLeaf<time_t, Fact>* it = tree->getFirst();
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
