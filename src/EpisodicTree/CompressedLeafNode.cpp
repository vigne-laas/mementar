#include <sstream>
#include <experimental/filesystem>

#include "mementar/EpisodicTree/CompressedLeafNode.h"
#include "mementar/Display.h"

namespace mementar
{

CompressedLeafNode::CompressedLeafNode(std::string directory, size_t order)
{
  directory_ = directory;
  order_ = order;

  last_tree_nb_leafs_ = 0;
  earlier_key_ = 0;

  loadStoredData();
  Context::loadContexts(contexts_, directory_);

  running_ = true;
  session_cleaner_ = std::move(std::thread(&CompressedLeafNode::clean, this));
}

CompressedLeafNode::~CompressedLeafNode()
{
  running_ = false;
  session_cleaner_.join();

  Context::storeContexts(contexts_, directory_);

  mut_.lock();
  Display::Info("Compress trees:");
  size_t nb_leafs = keys_.size();
  size_t leafs_cpt = 0;
  Display::Percent(0);

  for(auto tree : btree_childs_)
  {
    compressed_childs_.push_back(CompressedLeaf(tree, directory_));
    delete tree;
    Display::Percent((++leafs_cpt)*100/nb_leafs);
  }

  for(size_t i = 0; i < compressed_sessions_tree_.size(); i++)
  {
    if(compressed_sessions_tree_[i] != nullptr)
    {
      if(modified_[i])
        compressed_childs_[i] = std::move(CompressedLeaf(compressed_sessions_tree_[i], directory_));
      delete compressed_sessions_tree_[i];
    }
    Display::Percent((++leafs_cpt)*100/nb_leafs);
  }
  Display::Debug("");
  mut_.unlock();
}

void CompressedLeafNode::insert(const time_t& key, const Fact& data)
{
  mut_.lock_shared();
  if(keys_.size() == 0)
  {
    mut_.unlock_shared();
    createNewTreeChild(key);
    mut_.lock_shared();
    last_tree_nb_leafs_ = btree_childs_[0]->insert(key, data);
    contexts_[0].insert(data);
  }
  else
  {
    if(key < keys_[0])
    {
      std::cout << "[ERROR] try to insert fact in past that do not exist" << std::endl;
      return;
    }

    size_t index = getKeyIndex(key);

    if(index < compressed_childs_.size())
    {
      if((index == compressed_childs_.size() - 1) && (keys_.size() == compressed_childs_.size()) && (key > earlier_key_))
      {
        mut_.unlock_shared();
        createNewTreeChild(key);
        mut_.lock_shared();
        last_tree_nb_leafs_ = btree_childs_[0]->insert(key, data);
        contexts_[keys_.size()].insert(data);
      }
      else
      {
        mut_.unlock_shared();
        createSession(index);
        mut_.lock_shared();
        compressed_sessions_tree_[index]->insert(key, data);
        contexts_[index].insert(data);
        modified_[index] = true;
      }
    }
    else if(useNewTree())
    {
      mut_.unlock_shared();
      createNewTreeChild(key);
      mut_.lock_shared();
      last_tree_nb_leafs_ = btree_childs_[btree_childs_.size() - 1]->insert(key, data);
      contexts_[keys_.size() - 1].insert(data);

      //verify if a chld need to be compressed
      if(btree_childs_.size() > 2)
      {
        mut_.unlock_shared();
        compressFirst();
        mut_.lock_shared();
      }
    }
    else if(index - keys_.size() + 1 == 0) // if insert in more recent tree
    {
      last_tree_nb_leafs_ = btree_childs_[index - compressed_childs_.size()]->insert(key,data);
      contexts_[index].insert(data);
    }
    else
    {
      btree_childs_[index - compressed_childs_.size()]->insert(key,data);
      contexts_[index].insert(data);
    }
  }
  mut_.unlock_shared();

  if(earlier_key_ < key)
    earlier_key_ = key;
}

void CompressedLeafNode::remove(const time_t& key, const Fact& data)
{
  mut_.lock_shared();
  int index = getKeyIndex(key);
  if(index >= 0)
  {
    if((size_t)index < compressed_childs_.size())
    {
      mut_.unlock_shared();
      createSession(index);
      mut_.lock_shared();
      if(compressed_sessions_tree_[index]->remove(key, data))
      {
        modified_[index] = true;
        contexts_[index].remove(data);
      }
    }
    else
    {
      if(btree_childs_[index - compressed_childs_.size()]->remove(key, data))
        contexts_[index].remove(data);
    }
  }
  mut_.unlock_shared();
}

BtreeLeaf<time_t, Fact>* CompressedLeafNode::find(const time_t& key)
{
  BtreeLeaf<time_t, Fact>* res = nullptr;

  mut_.lock_shared();
  int index = getKeyIndex(key);
  if(index >= 0)
  {
    if((size_t)index < compressed_childs_.size())
    {
      mut_.unlock_shared();
      createSession(index);
      mut_.lock_shared();
      res = compressed_sessions_tree_[index]->find(key);
    }
    else
      res = btree_childs_[index - compressed_childs_.size()]->find(key);
  }
  mut_.unlock_shared();
  return res;
}

BtreeLeaf<time_t, Fact>* CompressedLeafNode::findNear(const time_t& key)
{
  BtreeLeaf<time_t, Fact>* res = nullptr;

  mut_.lock_shared();
  int index = getKeyIndex(key);
  if(index >= 0)
  {
    if((size_t)index < compressed_childs_.size())
    {
      mut_.unlock_shared();
      createSession(index);
      mut_.lock_shared();
      res = compressed_sessions_tree_[index]->findNear(key);
    }
    else
      res = btree_childs_[index - compressed_childs_.size()]->findNear(key);
  }
  mut_.unlock_shared();

  return res;
}

BtreeLeaf<time_t, Fact>* CompressedLeafNode::getFirst()
{
  BtreeLeaf<time_t, Fact>* res = nullptr;

  mut_.lock_shared();
  if(compressed_childs_.size())
  {
    mut_.unlock_shared();
    createSession(0);
    mut_.lock_shared();
    res = compressed_sessions_tree_[0]->getFirst();
  }
  else if(btree_childs_.size())
    res = btree_childs_[0]->getFirst();
  mut_.unlock_shared();

  return res;
}

BtreeLeaf<time_t, Fact>* CompressedLeafNode::getLast()
{
  BtreeLeaf<time_t, Fact>* res = nullptr;

  mut_.lock_shared();
  if(btree_childs_.size())
    res = btree_childs_[btree_childs_.size() - 1]->getLast();
  else if(compressed_childs_.size())
  {
    mut_.unlock_shared();
    createSession(compressed_childs_.size() - 1);
    mut_.lock_shared();
    res = compressed_sessions_tree_[compressed_childs_.size() - 1]->getLast();
  }
  mut_.unlock_shared();

  return res;
}

void CompressedLeafNode::display(time_t key)
{
  mut_.lock_shared();
  int index = getKeyIndex(key);

  if(index >= 0)
  {
    if((size_t)index < compressed_childs_.size())
      std::cout << compressed_childs_[index].getDirectoty() << std::endl;
    else
      btree_childs_[index - compressed_childs_.size()]->display();
  }
  mut_.unlock_shared();
}

void CompressedLeafNode::createNewTreeChild(const time_t& key)
{
  mut_.lock();
  btree_childs_.push_back(new Btree<time_t,Fact>(order_));
  keys_.push_back(key);
  contexts_.push_back(Context(key));
  mut_.unlock();
}

bool CompressedLeafNode::useNewTree()
{
  if(last_tree_nb_leafs_ >= 100000)
    return true;
  else
    return false;
}

int CompressedLeafNode::getKeyIndex(const time_t& key)
{
  int index = keys_.size() - 1;
  for(size_t i = 0; i < keys_.size(); i++)
  {
    if(key < keys_[i])
    {
      index = i - 1;
      break;
    }
  }
  return index;
}

void CompressedLeafNode::loadStoredData()
{
  Display::Info("Load compressed files:");
  size_t nb_file = std::distance(std::experimental::filesystem::directory_iterator(directory_), std::experimental::filesystem::directory_iterator{});
  size_t cpt_file = 0;
  Display::Percent(0);

  for(const auto& entry : std::experimental::filesystem::directory_iterator(directory_))
  {
    std::string complete_dir = entry.path();
    std::string dir = complete_dir.substr(directory_.size());
    if(dir[0] == '/')
      dir.erase(dir.begin());
    size_t dot_pose = dir.find(".");
    if(dot_pose != std::string::npos)
    {
      std::string ext = dir.substr(dot_pose + 1);
      std::string str_key = dir.substr(0, dot_pose);
      if(ext == "mlz")
      {
        time_t key;
        std::istringstream iss(str_key);
        iss >> key;
        insert(key, CompressedLeaf(key, complete_dir));

        Display::Debug(complete_dir);
      }
    }

    cpt_file++;
    Display::Percent(cpt_file*100/nb_file);
  }
  Display::Percent(100);
  Display::Debug("");

  if(compressed_childs_.size())
  {
    createSession(compressed_childs_.size() - 1);
    earlier_key_ = compressed_sessions_tree_[compressed_childs_.size() - 1]->getLast()->getKey();
  }
}

void CompressedLeafNode::insert(const time_t& key, const CompressedLeaf& leaf)
{
  mut_.lock();
  if((keys_.size() == 0) || (key > keys_[keys_.size() - 1]))
  {
    keys_.push_back(key);
    contexts_.push_back(Context(key));
    compressed_childs_.push_back(leaf);
    compressed_sessions_tree_.push_back(nullptr);
    compressed_sessions_timeout_.push_back(0);
    modified_.push_back(false);
  }
  else
  {
    for(size_t i = 0; i < keys_.size(); i++)
    {
      if(key < keys_[i])
      {
        keys_.insert(keys_.begin() + i, key);
        compressed_childs_.insert(compressed_childs_.begin() + i, leaf);
        contexts_.insert(contexts_.begin() + i, Context(key));
        compressed_sessions_tree_.insert(compressed_sessions_tree_.begin() + i, nullptr);
        compressed_sessions_timeout_.insert(compressed_sessions_timeout_.begin() + i, 0);
        modified_.insert(modified_.begin() + i, false);
        break;
      }
    }
  }
  mut_.unlock();
}

void CompressedLeafNode::compressFirst()
{
  if(btree_childs_.size() == 0)
    return;

  CompressedLeaf tmp(btree_childs_[0], directory_);

  mut_.lock();
  compressed_childs_.push_back(tmp);
  compressed_sessions_tree_.push_back(nullptr);
  compressed_sessions_timeout_.push_back(0);

  btree_childs_.erase(btree_childs_.begin());
  mut_.unlock();
}

void CompressedLeafNode::createSession(size_t index)
{
  mut_.lock();
  if(compressed_sessions_tree_[index] == nullptr)
    compressed_sessions_tree_[index] = compressed_childs_[index].getTree();

  compressed_sessions_timeout_[index] = std::time(0);
  mut_.unlock();
}

void CompressedLeafNode::clean()
{
  size_t rate = 1000000 / 1000; // clean rate / look up rate;
  size_t cpt = rate;
  while(running_ == true)
  {
    if(cpt-- == 0)
    {
      cpt = rate;
      time_t now = std::time(0);
      for(size_t i = 0; i < compressed_sessions_timeout_.size(); i++)
      {
        if((compressed_sessions_tree_[i] != nullptr) &&
          (std::difftime(now, compressed_sessions_timeout_[i]) > 30)) // session expire after 30s
        {
          mut_.lock();
          if(modified_[i])
            compressed_childs_[i] = std::move(CompressedLeaf(compressed_sessions_tree_[i], directory_));
          delete compressed_sessions_tree_[i];
          compressed_sessions_tree_[i] = nullptr;
          mut_.unlock();
        }

      }
    }
  }
}

} // mementar
