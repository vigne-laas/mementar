#ifndef BINARYMANAGER_H
#define BINARYMANAGER_H

#include <iostream>
#include <fstream>
#include <streambuf>
#include <vector>

namespace mementar
{

class BinaryManager
{
public:
  BinaryManager(std::string extension)
  {
    extension_ = extension;
  }

  void displayCompressionRate(size_t in_size, size_t out_size)
  {
    std::cout << "Compression rate : " << (1 - (out_size / (float)in_size)) * 100.0f << std::endl;
  }

  void saveToFile(std::vector<char>& data, const std::string& file_name)
  {
    std::ofstream outfile;
  	outfile.open(file_name + "." + extension_, std::ios::binary | std::ios::out);
    std::string str(data.begin(), data.end());
  	outfile.write(str.c_str(), str.length());
  	outfile.close();

    std::cout << "Saved into " << file_name << "." << extension_ << std::endl;
  }

  bool readBinaryFile(std::vector<char>& data, const std::string& file_name)
  {
    std::ifstream infile(file_name, std::ios::binary | std::ios::ate);
    std::streamsize size = infile.tellg();
    infile.seekg(0, std::ios::beg);

    data = std::vector<char>(size);
    if(infile.read(data.data(), size))
      return true;
    else
      return false;
  }
private:
  std::string extension_;
};

} // namespace mementar

#endif // BINARYMANAGER_H
