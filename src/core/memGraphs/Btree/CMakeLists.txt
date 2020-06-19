cmake_minimum_required (VERSION 3.5)
project (mementar)

add_compile_options(-std=c++1z -Wall)
link_libraries(stdc++fs)

include_directories(
  ../../../../include
)

add_library(mementar_dll_lib
  ../DoublyLinkedList/DllCargoNode.cpp
  ../DoublyLinkedList/DllLinkedElement.cpp
)

add_executable(btree tests/btree_test.cpp)
add_executable(max tests/max_insertion.cpp)
add_executable(cargo tests/cargo.cpp)
target_link_libraries(cargo mementar_dll_lib)
