#ifndef HEADER_H
#define HEADER_H

#include <vector>
#include <string>

struct File_t
{
  File_t(std::string name = "", size_t offset = 0, size_t size = 0)
  {
    path_ = name;
    name_ = name;
    while(name_.find("/") != std::string::npos)
      name_ = name_.substr(name_.find("/") + 1);
    offset_ = offset;
    size_ = size;
  }

  std::string name_;
  std::string path_;
  size_t offset_;
  size_t size_;
};

class Header
{
public:
  Header() {}

  File_t description_file_;
  std::vector<File_t> input_files_;

  std::string toString();

  size_t endodedSize();
  void encode(std::vector<char>& out);
  void decode(std::vector<char>& data);
};

#endif // HEADER_H
