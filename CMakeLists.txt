cmake_minimum_required (VERSION 3.5)
project (mementar)

add_compile_options(-std=c++14 -Wall)

include_directories(
  include
)

add_library(mementar_compression_lib
  src/compressing/LzCompress.cpp
  src/compressing/LzUncompress.cpp
  src/compressing/BitFileGenerator.cpp
  src/compressing/BitFileGetter.cpp
  src/compressing/Huffman.cpp
)

add_executable(compress src/tests/compress.cpp)
target_link_libraries(compress mementar_compression_lib)

add_executable(uncompress src/tests/uncompress.cpp)
target_link_libraries(uncompress mementar_compression_lib)
