#ifndef HEADER_H
#define HEADER_H

#include <vector>
#include <string>

struct File_t
{
  File_t(std::string name = "", size_t offset = 0, size_t size = 0)
  {
    name_ = name;
    offset_ = offset;
    size_ = size;
  }

  std::string name_;
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
