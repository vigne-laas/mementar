cmake_minimum_required (VERSION 3.5)
project (mementar)

add_compile_options(-std=c++1z -Wall)
link_libraries(stdc++fs)

include_directories(
  ../../include
)

add_library(mementar_compression_lib
  ../archiving_compressing/binaryManagement/BitFileGenerator.cpp
  ../archiving_compressing/binaryManagement/BitFileGetter.cpp
  ../archiving_compressing/compressing/LzCompress.cpp
  ../archiving_compressing/compressing/LzUncompress.cpp
  ../archiving_compressing/compressing/Huffman.cpp
  ../archiving_compressing/archiving/Header.cpp
  ../archiving_compressing/archiving/Archive.cpp
)

add_executable(btree tests/btree_test.cpp)

add_executable(max tests/max_insertion.cpp)

add_executable(episodic tests/episodic_tree.cpp)
target_link_libraries(episodic mementar_compression_lib)
