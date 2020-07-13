#include <vector>
#include <regex>
#include <experimental/filesystem>

#include "mementar/core/LtManagement/EpisodicTree/ArchivedLeaf.h"

#include "mementar/core/LtManagement/archiving_compressing/archiving/Archive.h"
#include "mementar/core/LtManagement/archiving_compressing/archiving/Header.h"
#include "mementar/core/LtManagement/archiving_compressing/compressing/LzUncompress.h"

namespace mementar
{

ArchivedLeaf::ArchivedLeaf(CompressedLeafNode* tree, size_t nb, const std::string& directory)
{
  if(tree == nullptr)
    return;

  key_ = tree->getFirst()->getKey();
  directory_ = directory + '/' + std::to_string(key_);

  std::vector<time_t> keys;
  std::vector<Context> contexts;
  std::vector<std::string> input_files;

  for(size_t i = 0; i < nb; i++)
  {
    if(i >= tree->compressed_childs_.size())
      break;

    keys.push_back(tree->keys_[i]);
    contexts.push_back(tree->contexts_[i]);
    input_files.push_back(tree->compressed_childs_[i].getDirectory() + ".mlz");
  }

  std::string context = Context::ContextsToString(contexts);

  Archive arch(context, input_files);

  std::vector<char> data = arch.load();

  arch.saveToFile(data, directory_);

  for(size_t i = 0; i < nb; i++)
  {
    if(i >= tree->compressed_childs_.size())
      break;

    std::experimental::filesystem::remove(tree->compressed_childs_[i].getDirectory() + ".mlz");
  }
}

ArchivedLeaf::ArchivedLeaf(const time_t& key, const std::string& directory)
{
  key_ = key;
  size_t dot_pose = directory.find(".");
  directory_ = directory.substr(0, dot_pose);
}

BplusTree<time_t, Event*>* ArchivedLeaf::getTree(size_t i)
{
  mementar::Archive arch;
  std::cout << "ArchivedLeaf::getTree READ BINARY FILE " << directory_ << ".mar" << std::endl;
  arch.readBinaryFile(directory_ + ".mar");
  mementar::Header header = arch.getHeader();

  std::string comp = arch.extractFile(i, header);

  LzUncompress lz;
  std::vector<char> comp_data(comp.begin(), comp.end());
  std::string out = lz.uncompress(comp_data);
  BplusTree<time_t, Event*>* tree = new BplusTree<time_t, Event*>();

  std::istringstream iss(out);
  std::string line;
  while(std::getline(iss, line))
  {
    Event* event = Event::deserializePtr(line);
    if(event != nullptr)
      tree->insert(event->getTime(), event);
  }

  return tree;
}

std::vector<Context> ArchivedLeaf::getContexts()
{
  mementar::Archive arch;
  std::cout << "ArchivedLeaf::getContexts READ BINARY FILE " << directory_ << ".mar" << std::endl;
  arch.readBinaryFile(directory_ + ".mar");
  mementar::Header header = arch.getHeader();

  std::string out = arch.extractDescription(header);

  return Context::StringToContext(out);
}

} // namespace mementar
