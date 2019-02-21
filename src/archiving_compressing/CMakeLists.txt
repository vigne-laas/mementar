cmake_minimum_required (VERSION 3.5)
project (mementar)

add_compile_options(-std=c++14 -Wall)

include_directories(
  ../../include/mementar
)

add_library(mementar_compression_lib
  binaryManagement/BitFileGenerator.cpp
  binaryManagement/BitFileGetter.cpp
  compressing/LzCompress.cpp
  compressing/LzUncompress.cpp
  compressing/Huffman.cpp
  archiving/Header.cpp
  archiving/Archive.cpp
)

add_executable(compress tests/compress.cpp)
target_link_libraries(compress mementar_compression_lib)

add_executable(uncompress tests/uncompress.cpp)
target_link_libraries(uncompress mementar_compression_lib)

add_executable(archive tests/archive.cpp)
target_link_libraries(archive mementar_compression_lib)
