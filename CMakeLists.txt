cmake_minimum_required (VERSION 3.5)
project (mementor)

add_compile_options(-std=c++14)

include_directories(
  include
)

add_library(mementor_compression_lib
  src/lz/LzCompress.cpp
  src/lz/LzUncompress.cpp
  src/lz/BitFileGenerator.cpp
  src/lz/BitFileGetter.cpp
  src/lz/Huffman.cpp
)

add_executable(compress src/compress.cpp)
target_link_libraries(compress mementor_compression_lib)

add_executable(uncompress src/uncompress.cpp)
target_link_libraries(uncompress mementor_compression_lib)
