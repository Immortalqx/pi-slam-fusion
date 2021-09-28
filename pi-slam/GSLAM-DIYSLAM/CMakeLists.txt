cmake_minimum_required(VERSION 2.8.6)
project(DIYSLAM)

if(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
endif()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

include(cmake/PICMake.cmake)

if(WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,-rpath,.:./libs:../libs:${CMAKE_INSTALL_PREFIX}/lib -std=c++11")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,-rpath,.:./libs:../libs:${CMAKE_INSTALL_PREFIX}/lib -std=c++11")
endif()

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=x86-64 -mtune=generic")#-march=x86-64 -mtune=generic -march=native
set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -Wl,--retain-symbols-file=${CMAKE_SOURCE_DIR}/cmake/static_export.sym -Wl,--version-script=${CMAKE_SOURCE_DIR}/cmake/shared_export.map")

INCLUDE_DIRECTORIES(src)

pi_add_target(diyslam SHARED src REQUIRED System GSLAM GLEW GLUT OpenGL Eigen3 OpenCV DEPENDENCY -lX11 -lopencv_video)#
pi_report_target()
pi_install(TARGETS diyslam LIB_DESTINATION lib)
