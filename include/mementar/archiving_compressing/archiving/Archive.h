#ifndef MEMENTAR_ARCHIVE_H
#define MEMENTAR_ARCHIVE_H

#include "mementar/archiving_compressing/archiving/Header.h"
#include "mementar/archiving_compressing/binaryManagement/BinaryManager.h"

namespace mementar
{

class Archive : public BinaryManager
{
public:
  Archive(std::string& description, std::vector<std::string>& files);
  Archive(const std::string& description, const Header& header);
  Archive();

  bool readBinaryFile(const std::string& file_name)
  {
    return BinaryManager::readBinaryFile(data_, file_name);
  }

  void load(std::vector<char>& out);
  void load(std::vector<char>& out, std::vector<std::vector<char> >& raw_datas);

  Header getHeader(std::vector<char>& data);
  Header getHeader() { return getHeader(data_); }

  std::string extractDescription(Header& head, std::vector<char>& data);
  std::string extractDescription(Header& head)
  {
    return extractDescription(head, data_);
  }
  std::string extractFile(size_t index, Header& head, std::vector<char>& data);
  std::string extractFile(size_t index, Header& head)
  {
    return extractFile(index, head, data_);
  }

private:
  Header header_;
  std::string description_;
  std::vector<char> data_;
};

} // namespace mementar

#endif // MEMENTAR_ARCHIVE_H
