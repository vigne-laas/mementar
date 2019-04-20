cmake_minimum_required (VERSION 3.5)
project (mementar)

add_compile_options(-std=c++14 -Wall)

include_directories(
  ../../include
)

add_executable(btree tests/btree_test.cpp)
