#ifndef MEMENTAR_LZUNCOMPRESS_H
#define MEMENTAR_LZUNCOMPRESS_H

#include "mementar/archiving_compressing/binaryManagement/BitFileGetter.h"
#include "mementar/archiving_compressing/binaryManagement/BinaryManager.h"

namespace mementar
{

class LzUncompress : public BinaryManager
{
public:
  LzUncompress();

  void uncompress(std::vector<char>& data, std::string& out);

private:
  size_t search_size_;
  size_t la_size_;

  BitFileGetter bit;

  int neededBitCount(size_t max_value);
};

} // namespace mementar

#endif // MEMENTAR_LZUNCOMPRESS_H
