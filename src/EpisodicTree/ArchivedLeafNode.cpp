#include "mementar/EpisodicTree/ArchivedLeafNode.h"

#include <sstream>
#include <experimental/filesystem>

#include "mementar/Display.h"

namespace mementar
{

ArchivedLeafNode::ArchivedLeafNode(const std::string& directory, size_t order)
{
  directory_ = directory;
  order_ = order;

  last_tree_nb_leafs_ = 0;
  earlier_key_ = 0;

  loadStoredData();

  running_ = true;
  session_cleaner_ = std::move(std::thread(&ArchivedLeafNode::clean, this));
}

ArchivedLeafNode::~ArchivedLeafNode()
{
  running_ = false;
  session_cleaner_.join();

  mut_.lock();
  Display::Info("Archive trees:");
  size_t nb_leafs = archived_sessions_tree_.size();
  size_t leafs_cpt = 0;
  Display::Percent(0);

  for(auto tree : archived_sessions_tree_)
  {
    if(tree != nullptr)
      delete tree;
    Display::Percent((++leafs_cpt)*100/nb_leafs);
  }

  Display::Debug("");
  mut_.unlock();
}

void ArchivedLeafNode::loadStoredData()
{
  Display::Info("Load archived files:");
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
      if(ext == "mar")
      {
        time_t key;
        std::istringstream iss(str_key);
        iss >> key;
        insert(key, ArchivedLeaf(key, complete_dir));

        Display::Debug(complete_dir);
      }
    }

    cpt_file++;
    Display::Percent(cpt_file*100/nb_file);
  }
  Display::Percent(100);
  Display::Debug("");

  if(archived_childs_.size())
  {
    createSession(archived_childs_.size() - 1);
    earlier_key_ = archived_sessions_tree_[archived_childs_.size() - 1]->getLast()->getKey();
  }
}

void ArchivedLeafNode::insert(const time_t& key, const ArchivedLeaf& leaf)
{
  mut_.lock();
  if((keys_.size() == 0) || (key > keys_[keys_.size() - 1]))
  {
    keys_.push_back(key);
    archived_childs_.push_back(leaf);
    archived_sessions_tree_.push_back(nullptr);
    archived_sessions_timeout_.push_back(0);
    modified_.push_back(false);
  }
  else
  {
    for(size_t i = 0; i < keys_.size(); i++)
    {
      if(key < keys_[i])
      {
        keys_.insert(keys_.begin() + i, key);
        archived_childs_.insert(archived_childs_.begin() + i, leaf);
        archived_sessions_tree_.insert(archived_sessions_tree_.begin() + i, nullptr);
        archived_sessions_timeout_.insert(archived_sessions_timeout_.begin() + i, 0);
        modified_.insert(modified_.begin() + i, false);
        break;
      }
    }
  }
  mut_.unlock();
}

void ArchivedLeafNode::createSession(size_t index)
{
  mut_.lock();
  if(archived_sessions_tree_[index] == nullptr)
    archived_sessions_tree_[index] = new CompressedLeafNodeSession(archived_childs_[index].getDirectoty());

  archived_sessions_timeout_[index] = std::time(0);
  mut_.unlock();
}

void ArchivedLeafNode::clean()
{
  size_t rate = 1000000 / 1000; // clean rate / look up rate;
  size_t cpt = rate;
  while(running_ == true)
  {
    if(cpt-- == 0)
    {
      cpt = rate;
      time_t now = std::time(0);
      for(size_t i = 0; i < archived_sessions_timeout_.size(); i++)
      {
        if((archived_sessions_tree_[i] != nullptr) &&
          (std::difftime(now, archived_sessions_timeout_[i]) > 30)) // session expire after 30s
        {
          mut_.lock();
          delete archived_sessions_tree_[i];
          archived_sessions_tree_[i] = nullptr;
          mut_.unlock();
        }

      }
    }
  }
}

} // namespace mementar
