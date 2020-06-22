#ifndef MEMENTAR_LZUNCOMPRESS_H
#define MEMENTAR_LZUNCOMPRESS_H

#include "mementar/core/LtManagement/archiving_compressing/binaryManagement/BitFileGetter.h"
#include "mementar/core/LtManagement/archiving_compressing/binaryManagement/BinaryManager.h"

namespace mementar
{

class LzUncompress : public BinaryManager
{
public:
  LzUncompress();

  std::string uncompress(const std::vector<char>& data);

private:
  int neededBitCount(size_t max_value);
};

} // namespace mementar

#endif // MEMENTAR_LZUNCOMPRESS_H
