cmake_minimum_required(VERSION 2.8)

project(Map2DFusion)

set(CMAKE_CXX_COMPILER "/usr/bin/g++-4.8")
set(CMAKE_C_COMPILER "/usr/bin/gcc-4.8")
set(OpenCV_DIR "/usr/local/opencv_2.4.9/share/OpenCV")

if(UNIX)
    list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake/linux64)
elseif(MSVC)
    list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake/vs2015x64)
else()
    list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake/mingw64)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
    
include(cmake/PICMake.cmake)


if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release) #Debug
endif()

message("-- Build type: " ${CMAKE_BUILD_TYPE})


if(UNIX)
    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -DPIL_LINUX")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPIL_LINUX")
elseif(MSVC)
    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   /DPIL_WINDOWS")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /DPIL_WINDOWS")
else()
    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -DPIL_MINGW")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPIL_MINGW")
endif()


if(MSVC)
    # definitions
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /DGLEW_BUILD /DGLEW_NO_GLU /DGLEW_STATIC")

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /DNOMINMAX")
    
    # about /MT and /MD
    #   http://www.cnblogs.com/cswuyg/archive/2012/02/03/2336424.html
    #   https://docs.microsoft.com/en-us/cpp/build/reference/md-mt-ld-use-run-time-library?view=vs-2019
    set(CompilerFlags
        CMAKE_CXX_FLAGS
        CMAKE_CXX_FLAGS_DEBUG
        CMAKE_CXX_FLAGS_RELEASE
        CMAKE_C_FLAGS
        CMAKE_C_FLAGS_DEBUG
        CMAKE_C_FLAGS_RELEASE
        CMAKE_C_FLAGS_RELWITHDEBINFO
        CMAKE_CXX_FLAGS_RELWITHDEBINFO
        CMAKE_C_FLAGS_MINSIZEREL
        CMAKE_CXX_FLAGS_MINSIZEREL
        CXX_FLAGS
        C_FLAGS
        )
    foreach(CompilerFlag ${CompilerFlags})
        string(REPLACE "/MD" "/MT" ${CompilerFlag} "${${CompilerFlag}}")
        message(STATUS "    CompilerFlag: ${${CompilerFlag}}")
    endforeach() 
    
    if(BUILD_WITH_DEBUG_INFO)
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /DEBUG /OPT:REF /OPT:ICF")
        set(CMAKE_EXE_LINKER_FLAGS    "${CMAKE_EXE_LINKER_FLAGS}    /DEBUG /OPT:REF /OPT:ICF")
        set(CMAKE_CXX_FLAGS           "${CMAKE_CXX_FLAGS}           /Zi")
        set(CMAKE_C_FLAGS             "${CMAKE_C_FLAGS}             /Zi")
    endif()
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DGLEW_BUILD -DGLEW_NO_GLU -DGLEW_STATIC -D_GNU_SOURCE")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")
    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -fPIC")

endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    if(UNIX)
        message("Build with -O3")

        #-DG2O_OPENMP -fopenmp# FIXME: OPENMP would cause slow optimization!!!
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,-rpath,.:lib:../lib")

        # set CPU type
        set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -march=x86-64 -mtune=generic")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=x86-64 -mtune=generic")

        # static link libs
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -static-libgcc -static-libstdc++")
        set(CMAKE_EXE_LINKER_FLAGS    "${CMAKE_EXE_LINKER_FLAGS}    -static-libgcc -static-libstdc++")
    
        # debug information
        if(BUILD_WITH_DEBUG_INFO)
            set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -g")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
        endif()
    endif()
endif()



pi_collect_packages(VERBOSE REQUIRED System OpenCV
                    MODULES OPENGL)

find_package(Poco REQUIRED Net) 
find_package(Qt4 REQUIRED)

if(QT4_FOUND)
    qt4_add_resources(qrc_mapresources Thirdpart/opmapcontrol/mapwidget/mapresources.qrc)
    qt4_wrap_ui(UI_OPM Thirdpart/opmapcontrol/mapwidget/mapripform.ui)
elseif(QT5_FOUND)
    qt5_add_resources(qrc_mapresources Thirdpart/opmapcontrol/mapwidget/mapresources.qrc)
    qt5_wrap_ui(UI_OPM Thirdpart/opmapcontrol/mapwidget/mapripform.ui)
endif()

INCLUDE_DIRECTORIES(src
                    PIL/src
                    Thirdpart
                    Thirdpart/opmapcontrol
                    Thirdpart/lua-5.1.5/src
                    Thirdpart/glew-1.13.0/include
                    ${PROJECT_BINARY_DIR})

#pi_add_target(gslamDB_net SHARED src/plugins/dbnet REQUIRED GSLAM Qt FFMPEG System)
pi_add_target(map2dfusion BIN
    src
    ${QRC_FILES} ${UI_OPM}
    ${qrc_mapresources}
    PIL/src/base 
    PIL/src/gui 
    PIL/src/hardware
    PIL/src/lua
    PIL/src/network 
    Thirdpart/opmapcontrol/core
    Thirdpart/opmapcontrol/internals
    Thirdpart/opmapcontrol/mapwidget
    Thirdpart/QGLViewer
    Thirdpart/lua-5.1.5/src
    Thirdpart/glew-1.13.0/src
    REQUIRED OpenCV Qt OpenGL Poco System)


pi_report_target()

MESSAGE(STATUS "INCLUDE_DIRECTORIES = ")
get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  MESSAGE(STATUS "   ${dir}")
endforeach()
message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
message(STATUS "CMAKE_C_FLAGS  : ${CMAKE_C_FLAGS}")


