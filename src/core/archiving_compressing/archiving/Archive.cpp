#include "mementar/core/archiving_compressing/archiving/Archive.h"

#include "mementar/core/archiving_compressing/compressing/LzCompress.h"
#include "mementar/core/archiving_compressing/compressing/LzUncompress.h"
#include "mementar/core/archiving_compressing/compressing/Huffman.h"

namespace mementar
{

Archive::Archive(std::string& description, std::vector<std::string>& files) : BinaryManager("mar")
{
  description_ = description;
  header_.description_file_ = File_t("description");
  for(const auto& file : files)
    header_.input_files_.push_back(File_t(file));

  //set header_ offset
  size_t header_size = header_.endodedSize();
  header_.description_file_.offset_ = header_size;
  for(auto& file : header_.input_files_)
    file.offset_ = header_size;
}

Archive::Archive(const std::string& description, const Header& header) : BinaryManager("mar")
{
  description_ = description;
  header_.description_file_ = File_t("description");
  header_.input_files_ = header.input_files_;

  //set header_ offset
  size_t header_size = header_.endodedSize();
  header_.description_file_.offset_ = header_size;
  for(auto& file : header_.input_files_)
    file.offset_ = header_size;
}

Archive::Archive() : BinaryManager("mar")
{}

void Archive::load(std::vector<char>& out)
{
  size_t offset = 0;

  LzCompress lz_comp;
  std::vector<char> out_vect = lz_comp.compress(description_);
  out.insert(out.end(), out_vect.begin(), out_vect.end());
  offset += out_vect.size();
  header_.description_file_.size_ = out_vect.size();
  out_vect = std::vector<char>();

  std::vector<std::string> input_files;
  Huffman huff;
  for(const auto& file : header_.input_files_)
  {
    auto str = huff.readBinaryFile(file.path_);
    input_files.push_back(str);
    huff.analyse(str);
  }

  huff.generateCode();
  huff.getTreeCode(out_vect);
  out.insert(out.end(), out_vect.begin(), out_vect.end());
  offset += out_vect.size();
  out_vect = std::vector<char>();

  for(size_t i = 0; i < input_files.size(); i++)
  {
    huff.getDataCode(input_files[i], out_vect);
    out.insert(out.end(), out_vect.begin(), out_vect.end());
    header_.input_files_[i].offset_ += offset;
    offset += out_vect.size();
    header_.input_files_[i].size_ = out_vect.size();
    out_vect = std::vector<char>();
  }

  header_.encode(out_vect);
  out.insert(out.begin(), out_vect.begin(), out_vect.end());
}

void Archive::load(std::vector<char>& out, std::vector<std::string>& raw_datas)
{
  if(raw_datas.size() != header_.input_files_.size())
  {
    std::cout << "ERROR" <<std::endl;
    return;
  }

  size_t offset = 0;

  LzCompress lz_comp;
  std::vector<char> out_vect = lz_comp.compress(description_);
  out.insert(out.end(), out_vect.begin(), out_vect.end());
  offset += out_vect.size();
  header_.description_file_.size_ = out_vect.size();
  out_vect = std::vector<char>();

  Huffman huff;
  for(auto& raw_data : raw_datas)
    huff.analyse(raw_data);

  huff.generateCode();
  huff.getTreeCode(out_vect);
  out.insert(out.end(), out_vect.begin(), out_vect.end());
  offset += out_vect.size();
  out_vect = std::vector<char>();

  for(size_t i = 0; i < raw_datas.size(); i++)
  {
    huff.getDataCode(raw_datas[i], out_vect);
    out.insert(out.end(), out_vect.begin(), out_vect.end());
    header_.input_files_[i].offset_ += offset;
    offset += out_vect.size();
    header_.input_files_[i].size_ = out_vect.size();
    out_vect = std::vector<char>();
  }

  header_.encode(out_vect);
  out.insert(out.begin(), out_vect.begin(), out_vect.end());
}

Header Archive::getHeader(std::vector<char>& data)
{
  header_.decode(data);
  return header_;
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
  out = huff.getFile(tmp_data);

  return out;
}

} // namespace mementar
