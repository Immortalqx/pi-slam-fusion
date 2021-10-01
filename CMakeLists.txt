cmake_minimum_required(VERSION 2.8)
project(PiSLAMFusion)

set(CMAKE_CXX_STANDARD 11)

add_executable(PiSLAMFusion src/main.cpp)

target_link_libraries(${PROJECT_NAME})



#add_subdirectory(Map2DFusion)
#add_subdirectory(pi-slam)
