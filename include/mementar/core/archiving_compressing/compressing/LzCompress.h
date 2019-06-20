#ifndef MEMENTAR_LZCOMPRESS_H
#define MEMENTAR_LZCOMPRESS_H

#include <string>
#include <vector>

#include "mementar/core/archiving_compressing/binaryManagement/BitFileGenerator.h"
#include "mementar/core/archiving_compressing/binaryManagement/BinaryManager.h"

namespace mementar
{

class LzCompress : public BinaryManager
{
public:
  LzCompress(size_t search_size = 2048, size_t la_size = 64);

  void compress(std::string& in, std::vector<char>& out);

private:
  size_t search_size_;
  size_t search_size_1_;
  size_t la_size_;
  size_t la_size_1_;

  BitFileGenerator bit;

  int neededBitCount(size_t max_value);
};

} // namespace mementar

#endif // MEMENTAR_LZCOMPRESS_H
