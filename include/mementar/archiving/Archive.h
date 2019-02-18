#ifndef ARCHIVE_H
#define ARCHIVE_H

#include "mementar/archiving/Header.h"
#include "mementar/binaryManagement/BinaryManager.h"

class Archive : public BinaryManager
{
public:
  Archive(std::string& description, std::vector<std::string>& files);
  Archive();

  void load(std::vector<char>& out);
  Header getHeader(std::vector<char>& data);

private:
  Header header;
  std::string description_;
};

#endif // ARCHIVE_H
