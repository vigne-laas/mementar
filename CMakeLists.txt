cmake_minimum_required (VERSION 3.5)
project (mementar)

add_compile_options(-std=c++14 -Wall)

include_directories(
  include
)

add_library(mementar_compression_lib
  src/binaryManagement/BitFileGenerator.cpp
  src/binaryManagement/BitFileGetter.cpp
  src/compressing/LzCompress.cpp
  src/compressing/LzUncompress.cpp
  src/compressing/Huffman.cpp
  src/archiving/Header.cpp
  src/archiving/Archive.cpp
)

add_executable(compress src/tests/compress.cpp)
target_link_libraries(compress mementar_compression_lib)

add_executable(uncompress src/tests/uncompress.cpp)
target_link_libraries(uncompress mementar_compression_lib)

add_executable(archive src/tests/archive.cpp)
target_link_libraries(archive mementar_compression_lib)
