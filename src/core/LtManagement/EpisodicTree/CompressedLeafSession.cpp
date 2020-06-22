#include "mementar/core/LtManagement/EpisodicTree/CompressedLeafSession.h"

#include <regex>
#include <sstream>

#include "mementar/core/LtManagement/archiving_compressing/compressing/LzUncompress.h"

namespace mementar
{

CompressedLeafSession::CompressedLeafSession(const time_t& key, size_t index)
{
  key_ = key;
  index_ = index;
}

Btree<time_t, Event*>* CompressedLeafSession::getTree(Header& header, Archive& arch)
{
  std::string comp = arch.extractFile(index_, header);

  LzUncompress lz;
  std::vector<char> comp_data(comp.begin(), comp.end());
  std::string out = lz.uncompress(comp_data);
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

std::vector<char> CompressedLeafSession::getRawData(Header& header, Archive& arch)
{
  std::string comp = arch.extractFile(index_, header);
  return std::vector<char>(comp.begin(), comp.end());
}

} // namespace mementar
