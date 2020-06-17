#include "mementar/core/EpisodicTree/CompressedLeafNodeSession.h"

#include "mementar/core/utility/Display.h"
#include "mementar/core/archiving_compressing/compressing/LzCompress.h"

namespace mementar
{

CompressedLeafNodeSession::CompressedLeafNodeSession(const std::string& file_name)
{
  file_name_ = file_name;

  earlier_key_ = 0;

  loadData();
}

CompressedLeafNodeSession::~CompressedLeafNodeSession()
{
  mut_.lock();
  std::string description = Context::ContextsToString(contexts_);

  Archive arch(description, header_);

  std::vector<std::vector<char> > raw_datas;
  for(size_t i = 0; i < childs_.size(); i++)
  {
    if(sessions_tree_[i] != nullptr)
    {
      if(modified_[i])
        raw_datas.push_back(treeToRaw(i));
      else
        raw_datas.push_back(childs_[i].getRawData(header_, arch_));
    }
    else
      raw_datas.push_back(childs_[i].getRawData(header_, arch_));
  }

  std::vector<char> data;
  arch.load(data, raw_datas);

  arch.saveToFile(data, file_name_);

  mut_.unlock();
}

int CompressedLeafNodeSession::getKeyIndex(const time_t& key)
{
  int index = contexts_.size() - 1;
  for(size_t i = 0; i < contexts_.size(); i++)
  {
    if(key < contexts_[i].getKey())
    {
      index = i - 1;
      break;
    }
  }
  return index;
}

void CompressedLeafNodeSession::insert(const time_t& key, const Fact& data)
{
  mut_.lock_shared();
  if(contexts_.size() == 0)
  {
    Display::Error("Can not insert in empty session");
  }
  else
  {
    if(key < contexts_[0].getKey())
    {
      Display::Error("try to insert fact in past that do not exist");
      return;
    }

    size_t index = getKeyIndex(key);
    mut_.unlock_shared();
    createSession(index);
    mut_.lock_shared();
    sessions_tree_[index]->insert(key, data);
    contexts_[index].insert(data);
    modified_[index] = true;
  }

  mut_.unlock_shared();
}

bool CompressedLeafNodeSession::remove(const time_t& key, const Fact& data)
{
  bool res = false;
  mut_.lock_shared();
  int index = getKeyIndex(key);
  if(index >= 0)
  {
    mut_.unlock_shared();
    createSession(index);
    mut_.lock_shared();
    if(sessions_tree_[index]->remove(key, data))
    {
      modified_[index] = true;
      contexts_[index].remove(data);
      res = true;
    }
  }
  mut_.unlock_shared();
  return res;
}

BtreeLeaf<time_t, Fact>* CompressedLeafNodeSession::find(const time_t& key)
{
  BtreeLeaf<time_t, Fact>* res = nullptr;

  mut_.lock_shared();
  int index = getKeyIndex(key);
  if(index >= 0)
  {
    mut_.unlock_shared();
    createSession(index);
    mut_.lock_shared();
    res = sessions_tree_[index]->find(key);
  }
  mut_.unlock_shared();
  return res;
}

BtreeLeaf<time_t, Fact>* CompressedLeafNodeSession::findNear(const time_t& key)
{
  BtreeLeaf<time_t, Fact>* res = nullptr;

  mut_.lock_shared();
  int index = getKeyIndex(key);
  if(index >= 0)
  {
    mut_.unlock_shared();
    createSession(index);
    mut_.lock_shared();
    res = sessions_tree_[index]->findNear(key);
  }
  mut_.unlock_shared();

  return res;
}

BtreeLeaf<time_t, Fact>* CompressedLeafNodeSession::getFirst()
{
  BtreeLeaf<time_t, Fact>* res = nullptr;

  createSession(0);
  mut_.lock_shared();
  res = sessions_tree_[0]->getFirst();
  mut_.unlock_shared();

  return res;
}

BtreeLeaf<time_t, Fact>* CompressedLeafNodeSession::getLast()
{
  BtreeLeaf<time_t, Fact>* res = nullptr;

  createSession(childs_.size() - 1);
  mut_.lock_shared();
  res = sessions_tree_[childs_.size() - 1]->getLast();
  mut_.unlock_shared();

  return res;
}

void CompressedLeafNodeSession::loadData()
{
  arch_.readBinaryFile(file_name_ + ".mar");
  header_ = arch_.getHeader();

  std::string description = arch_.extractDescription(header_);
  contexts_ = Context::StringToContext(description);

  //assume as ordered ?
  for(size_t i = 0; i < contexts_.size(); i++)
  {
    childs_.push_back(CompressedLeafSession(contexts_[i].getKey(), i));
    sessions_tree_.push_back(nullptr);
    modified_.push_back(false);
  }

  if(childs_.size())
  {
    createSession(childs_.size() - 1);
    earlier_key_ = sessions_tree_[childs_.size() - 1]->getLast()->getKey();
  }
}

void CompressedLeafNodeSession::createSession(size_t index)
{
  mut_.lock();
  if(sessions_tree_[index] == nullptr)
    sessions_tree_[index] = childs_[index].getTree(header_, arch_);
  mut_.unlock();
}

std::vector<char> CompressedLeafNodeSession::treeToRaw(size_t index)
{
  std::string res;

  std::vector<Fact> tmp_data;
  BtreeLeaf<time_t, Fact>* it = sessions_tree_[index]->getFirst();
  while(it != nullptr)
  {
    tmp_data = it->getData();
    for(auto& data : tmp_data)
      res += "[" + std::to_string(it->getKey()) + "]" + data.toString() + "\n";
    it = static_cast<BtreeLeaf<time_t, Fact>*>(it->getNextNode());
  }

  mementar::LzCompress lz_comp;
  std::vector<char> res_vect;
  lz_comp.compress(res, res_vect);

  return res_vect;
}

} // namespace mementar
