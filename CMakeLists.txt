cmake_minimum_required(VERSION 2.8)
project(PiSLAMFusion)

# TODO should remove
set(CMAKE_CXX_COMPILER "/usr/bin/g++-4.8")
set(CMAKE_C_COMPILER "/usr/bin/gcc-4.8")
set(OpenCV_DIR "/usr/local/opencv_2.4.9/share/OpenCV")
# TODO should remove

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake/linux64)

include(cmake/PICMake.cmake)

# set default built type to Release
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()
message("-- Build type: " ${CMAKE_BUILD_TYPE})


# set compile options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DGLEW_BUILD -DGLEW_NO_GLU -DGLEW_STATIC -D_GNU_SOURCE")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   -DPIL_LINUX")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPIL_LINUX")

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    message("Build with -O3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DG2O_OPENMP -fopenmp")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,-rpath,.:lib:../lib")

    # set CPU type
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   -march=x86-64 -mtune=generic")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=x86-64 -mtune=generic")

    # static link libs
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -static-libgcc -static-libstdc++")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}    -static-libgcc -static-libstdc++")

    # debug information
    if (BUILD_WITH_DEBUG_INFO)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   -g")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
    endif ()
endif ()

pi_collect_packages(VERBOSE REQUIRED System MODULES # System dependency -dl is essential
        Eigen3                          # Dependency for Optimizer
        Qt OPENGL GLEW GLUT QGLViewer   # Dependency for GUI
        # FIXME: opencv.cmake not found!
        OpenCV                          # Dependency for IO
        )

find_package(Poco REQUIRED Net)
find_package(Qt4 REQUIRED)

if (QT4_FOUND)
    qt4_add_resources(QRC_FILES pi-slam/gui/gui_res.qrc)
    qt4_add_resources(qrc_mapresources Map2DFusion/Thirdpart/opmapcontrol/mapwidget/mapresources.qrc)
    qt4_wrap_ui(UI_OPM Map2DFusion/Thirdpart/opmapcontrol/mapwidget/mapripform.ui)
elseif (QT5_FOUND)
    qt5_add_resources(QRC_FILES pi-slam/gui/gui_res.qrc)
    qt5_add_resources(qrc_mapresources Map2DFusion/Thirdpart/opmapcontrol/mapwidget/mapresources.qrc)
    qt5_wrap_ui(UI_OPM Map2DFusion/Thirdpart/opmapcontrol/mapwidget/mapripform.ui)
endif ()

include_directories(
        pi-slam/gui
        pi-slam/GSLAM
        pi-slam/GSLAM-DIYSLAM/src
        ${CMAKE_CURRENT_LIST_DIR}

        Map2DFusion/src
        Map2DFusion/PIL/src
        Map2DFusion/Thirdpart
        Map2DFusion/Thirdpart/opmapcontrol
        Map2DFusion/Thirdpart/lua-5.1.5/src
        Map2DFusion/Thirdpart/glew-1.13.0/include
        ${PROJECT_BINARY_DIR}
)

# build of PI-SLAM
pi_add_target(PiSLAMFusion BIN
        src
        pi-slam/gui
        pi-slam/tests
        pi-slam/GSLAM-DIYSLAM/src
        ${QRC_FILES}

        cmake/glew
        cmake/QGLViewer

        # 目前的问题：GSLAM里面的SE3.h与PIL里面的冲突了，
        Map2DFusion/src
        ${UI_OPM}
        ${qrc_mapresources}
        Map2DFusion/PIL/src/base
        Map2DFusion/PIL/src/gui
        Map2DFusion/PIL/src/hardware
        Map2DFusion/PIL/src/lua
        Map2DFusion/PIL/src/network
        Map2DFusion/Thirdpart/opmapcontrol/core
        Map2DFusion/Thirdpart/opmapcontrol/internals
        Map2DFusion/Thirdpart/opmapcontrol/mapwidget
        Map2DFusion/Thirdpart/lua-5.1.5/src

        #        Map2DFusion/Thirdpart/QGLViewer
        #        Map2DFusion/Thirdpart/glew-1.13.0/src

        REQUIRED QGLViewer Qt GLEW GLUT OPENGL Eigen3 System Poco
        MODULES OpenCV
        )

pi_report_target()


MESSAGE(STATUS "INCLUDE_DIRECTORIES = ")
get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach (dir ${dirs})
    MESSAGE(STATUS "   ${dir}")
endforeach ()
message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
message(STATUS "CMAKE_C_FLAGS  : ${CMAKE_C_FLAGS}")


