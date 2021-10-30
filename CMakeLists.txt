cmake_minimum_required(VERSION 2.8)

project(PI-SLAM-Fusion)


list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
include(cmake/PICMake.cmake)


# set default built type to Release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
message("-- Build type: " ${CMAKE_BUILD_TYPE})


# set compile options
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -DPIL_LINUX")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPIL_LINUX")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DGLEW_BUILD -DGLEW_NO_GLU -DGLEW_STATIC -D_GNU_SOURCE")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DG2O_OPENMP -fopenmp")

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    message("Build with -O3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif()

include_directories(GSLAM
                    GSLAM-DIYSLAM/src
                    PIL/src
                    thirdparty
                    thirdparty/opmapcontrol
                    cmake/glew
                    ${CMAKE_CURRENT_LIST_DIR})

 #set(OpenCV_DIR “/usr/local/opencv2_4/share/OpenCV ”)
 

# build of PI-SLAM
pi_collect_packages(VERBOSE REQUIRED System MODULES # System dependency -dl is essential
                    Eigen3                          # Dependency for Optimizer
                    Qt OPENGL GLEW GLUT QGLViewer   # Dependency for GUI
                    OpenCV                          # Dependency for IO
                    )

find_package(Poco REQUIRED Net)
find_package(Qt4 REQUIRED)

if(QT4_FOUND)
    qt4_add_resources(QRC_FILES gui/gui_res.qrc)
elseif(QT5_FOUND)
    qt5_add_resources(QRC_FILES gui/gui_res.qrc)
endif()


# build of PI-SLAM
pi_add_target(pislamfusion BIN
    gui tests
    GSLAM-DIYSLAM/src
    ${QRC_FILES}
    PIL/src/base
    PIL/src/gui
    PIL/src/hardware
    PIL/src/network
    cmake/glew cmake/QGLViewer
    REQUIRED QGLViewer Qt GLEW GLUT OPENGL Eigen3 System
    MODULES OpenCV)

pi_report_target()


MESSAGE(STATUS "INCLUDE_DIRECTORIES = ")
get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
    MESSAGE(STATUS "   ${dir}")
endforeach()
message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
message(STATUS "CMAKE_C_FLAGS  : ${CMAKE_C_FLAGS}")

