#include <vector>
#include <regex>
#include <sstream>

#include "mementar/core/EpisodicTree/CompressedLeaf.h"

#include "mementar/core/archiving_compressing/compressing/LzCompress.h"
#include "mementar/core/archiving_compressing/compressing/LzUncompress.h"

namespace mementar
{

CompressedLeaf::CompressedLeaf(Btree<time_t, Event*>* tree, const std::string& directory)
{
  if(tree == nullptr)
    return;

  key_ = tree->getFirst()->getKey();
  directory_ = directory + '/' + std::to_string(key_);

  LzCompress lz_comp;
  std::string in = treeToString(tree);
  std::vector<char> out_vect = lz_comp.compress(in);

  lz_comp.displayCompressionRate(in.size(), out_vect.size());
  lz_comp.saveToFile(out_vect, directory_);
}

CompressedLeaf::CompressedLeaf(const time_t& key, const std::string& directory)
{
  key_ = key;
  size_t dot_pose = directory.find(".");
  directory_ = directory.substr(0, dot_pose);
}

Btree<time_t, Event*>* CompressedLeaf::getTree()
{
  LzUncompress lz;
  std::vector<char> data;
  if(lz.readBinaryFile(data, directory_ + ".mlz"))
  {
    std::string out = lz.uncompress(data);
    Btree<time_t, Event*>* tree = new Btree<time_t, Event*>();

    std::regex regex("\\[(\\d+)\\](\\w+)\\s*\\|\\s*(\\w+)\\s*\\|\\s*(\\w+)");
    std::smatch match;

    std::istringstream iss(out);
    std::string line;
    while(std::getline(iss, line))
    {
      if(std::regex_match(line, match, regex))
      {
        time_t key;
        std::istringstream iss(match[1].str());
        iss >> key;
        Event* event = new Event(Fact(match[2].str(), match[3].str(), match[4].str()), key);

        tree->insert(event->getTime(), event);
      }
    }

    return tree;
  }

  return nullptr;
}

std::string CompressedLeaf::treeToString(Btree<time_t, Event*>* tree)
{
  std::string res;
  std::vector<Event*> tmp_data;
  BtreeLeaf<time_t, Event*>* it = tree->getFirst();
  while(it != nullptr)
  {
    tmp_data = it->getData();
    for(auto& data : tmp_data)
      res += "[" + std::to_string(it->getKey()) + "]" + data->Fact::toString() + "\n";
    it = static_cast<BtreeLeaf<time_t, Event*>*>(it->getNextNode());
  }

  return std::move(res);
}

} // namespace mementar
