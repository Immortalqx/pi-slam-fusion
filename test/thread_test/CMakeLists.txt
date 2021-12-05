cmake_minimum_required(VERSION 3.20)
project(thread_test)

set(CMAKE_CXX_STANDARD 11)

link_libraries(pthread)

add_executable(thread_test main.cpp)
