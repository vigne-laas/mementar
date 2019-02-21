#include "archiving_compressing/archiving/Archive.h"

#include "archiving_compressing/compressing/LzCompress.h"
#include "archiving_compressing/compressing/LzUncompress.h"
#include "archiving_compressing/compressing/Huffman.h"

Archive::Archive(std::string& description, std::vector<std::string>& files) : BinaryManager("mar")
{
  description_ = description;
  header.description_file_ = File_t("description");
  for(const auto& file : files)
    header.input_files_.push_back(File_t(file));

  //set header offset
  size_t header_size = header.endodedSize();
  header.description_file_.offset_ = header_size;
  for(auto& file : header.input_files_)
    file.offset_ = header_size;
}

Archive::Archive() : BinaryManager("mar")
{}

void Archive::load(std::vector<char>& out)
{
  std::vector<char> out_vect;
  size_t offset = 0;

  LzCompress lz_comp;
  lz_comp.compress(description_, out_vect);
  out.insert(out.end(), out_vect.begin(), out_vect.end());
  offset += out_vect.size();
  header.description_file_.size_ = out_vect.size();
  out_vect = std::vector<char>();

  std::vector<std::vector<char> > input_files;
  Huffman huff;
  for(const auto& file : header.input_files_)
  {
    std::vector<char> in_vect;
    huff.readBinaryFile(in_vect, file.path_);
    input_files.push_back(in_vect);
    huff.analyse(in_vect);
  }

  huff.generateTree();
  huff.getTreeCode(out_vect);
  out.insert(out.end(), out_vect.begin(), out_vect.end());
  offset += out_vect.size();
  out_vect = std::vector<char>();

  for(size_t i = 0; i < input_files.size(); i++)
  {
    huff.getDataCode(input_files[i], out_vect);
    out.insert(out.end(), out_vect.begin(), out_vect.end());
    header.input_files_[i].offset_ += offset;
    offset += out_vect.size();
    header.input_files_[i].size_ = out_vect.size();
    out_vect = std::vector<char>();
  }

  header.encode(out_vect);
  out.insert(out.begin(), out_vect.begin(), out_vect.end());
}

Header Archive::getHeader(std::vector<char>& data)
{
  header.decode(data);
  return header;
}

std::string Archive::extractDescription(Header& head, std::vector<char>& data)
{
  std::string out;
  LzUncompress lz;
  std::vector<char> tmp_data(data.begin() + head.description_file_.offset_, data.begin() + head.description_file_.offset_ + head.description_file_.size_);
  lz.uncompress(tmp_data, out);
  return out;
}

std::string Archive::extractFile(size_t index, Header& head, std::vector<char>& data)
{
  std::string out;
  Huffman huff;

  std::vector<char> tree_data(data.begin() + head.description_file_.offset_ + head.description_file_.size_, data.end());
  huff.setTree(tree_data);
  std::vector<char> tmp_data(data.begin() + head.input_files_[index].offset_, data.begin() + head.input_files_[index].offset_ + head.input_files_[index].size_);
  huff.getFile(tmp_data, out);

  return out;
}
