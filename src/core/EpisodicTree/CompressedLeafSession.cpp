#include "mementar/core/EpisodicTree/CompressedLeafSession.h"

#include <regex>

#include "mementar/core/archiving_compressing/compressing/LzUncompress.h"

namespace mementar
{

CompressedLeafSession::CompressedLeafSession(const time_t& key, size_t index)
{
  key_ = key;
  index_ = index;
}

LinkedBtree<time_t>* CompressedLeafSession::getTree(Header& header, Archive& arch)
{
  std::string comp = arch.extractFile(index_, header);

  std::string out;
  LzUncompress lz;
  std::vector<char> comp_data(comp.begin(), comp.end());
  lz.uncompress(comp_data, out);
  LinkedBtree<time_t>* tree = new LinkedBtree<time_t>();

  std::regex regex("\\[(\\d+)\\](\\w+)\\|(\\w+)\\|(\\w+)");
  std::smatch match;

  std::istringstream iss(out);
  std::string line;
  while(std::getline(iss, line))
  {
    if(std::regex_match(line, match, regex))
    {
      LinkedFact* fact = new LinkedFact(match[2].str(), match[3].str(), match[4].str());
      time_t key;
      std::istringstream iss(match[1].str());
      iss >> key;

      tree->insert(key, fact);
    }
  }

  return tree;
}

std::vector<char> CompressedLeafSession::getRawData(Header& header, Archive& arch)
{
  std::string comp = arch.extractFile(index_, header);
  return std::vector<char>(comp.begin(), comp.end());
}

} // namespace mementar
