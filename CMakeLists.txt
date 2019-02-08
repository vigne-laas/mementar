cmake_minimum_required (VERSION 3.5)
project (mementar)

add_compile_options(-std=c++14)

include_directories(
  include
)

add_library(mementar_compression_lib
  src/lz/LzCompress.cpp
  src/lz/LzUncompress.cpp
  src/lz/BitFileGenerator.cpp
  src/lz/BitFileGetter.cpp
  src/lz/Huffman.cpp
)

add_executable(compress src/compress.cpp)
target_link_libraries(compress mementar_compression_lib)

add_executable(uncompress src/uncompress.cpp)
target_link_libraries(uncompress mementar_compression_lib)
