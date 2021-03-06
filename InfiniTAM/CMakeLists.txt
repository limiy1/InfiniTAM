cmake_minimum_required(VERSION 2.8)
project(InfiniTAM)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

macro(CANONIFY_BOOL var)
  if (${var})
    set(${var} TRUE)
  else()
    set(${var} FALSE)
  endif()
endmacro()

#Define Glut directory
set(GLUT_INCLUDE_DIR "C:/Program Files/freeglut/include")
set(GLUT_glut_LIBRARY "C:/Program Files/freeglut/lib/freeglutd.lib")
find_package(GLUT REQUIRED)
message("GLUT_INCLUDE_DIR - ${GLUT_INCLUDE_DIR}")
message("GLUT_LIBRARIES - ${GLUT_LIBRARIES}")
find_package(OpenGL REQUIRED)

find_package(CUDA QUIET)
CANONIFY_BOOL(CUDA_FOUND)
message(STATUS "CUDA found: " ${CUDA_FOUND})

#set(OPEN_NI_ROOT "C:/Program Files/OpenNI2")
find_package(OpenNI QUIET)
CANONIFY_BOOL(OPENNI_FOUND)
message(STATUS "OpenNI found: " ${OPENNI_FOUND})

find_package(libuvc QUIET)
CANONIFY_BOOL(libuvc_FOUND)
message(STATUS "libuvc found: ${libuvc_FOUND}")

#set(RealSense_INCLUDE_DIR "C:/installpack/librealsense-master/include")
#set(RealSense_LIBRARY "C:/installpack/librealsense-master/bin/x64")
set(REALSENSE_ROOT "C:/installpack/librealsense-master")
find_package(RealSense QUIET)
CANONIFY_BOOL(REALSENSE_FOUND)
message(STATUS "librealsense found: ${REALSENSE_FOUND}")

find_package(PNG QUIET)
CANONIFY_BOOL(PNG_FOUND)
message(STATUS "libpng found: ${PNG_FOUND}")

OPTION(WITH_CUDA "Build with CUDA support?" ${CUDA_FOUND})
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)
OPTION(WITH_REALSENSE "Build with Intel RealSense support?" ${REALSENSE_FOUND})

IF(MSVC_IDE)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
  add_definitions(-DUSING_CMAKE=1)
ELSE(MSVC_IDE)
  set(CFLAGS_WARN "-Wall -Wextra -Wno-unused-parameter -Wno-strict-aliasing")
  set(CMAKE_CXX_FLAGS "-fPIC -O3 -march=native ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
  #set(CMAKE_CXX_FLAGS "-fPIC -g ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
ENDIF(MSVC_IDE)

if(APPLE)
  set(CUDA_HOST_COMPILER /usr/bin/clang)
endif(APPLE)

IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++ -Wno-deprecated-declarations -Wno-unused-function")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libstdc++")
ENDIF()

#Define kinect include localtion
set(MSKINECTAPI_INCLUDE_DIR "C:/Program Files/Microsoft SDKs/Kinect/v2.0_1409/inc")
set(MSKINECTAPI_LIB "C:/Program Files/Microsoft SDKs/Kinect/v2.0_1409/Lib/x64/Kinect20.lib")
set(MSKINECTAPI_FOUND 1)
add_subdirectory(ORUtils)
add_subdirectory(ITMLib)
add_subdirectory(Utils)
add_subdirectory(Engine)

IF(WITH_CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})
ELSE()
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
ENDIF()

add_executable(InfiniTAM_cli InfiniTAM_cli.cpp)
target_link_libraries(InfiniTAM_cli Engine)
target_link_libraries(InfiniTAM_cli Utils)
add_executable(InfiniTAM InfiniTAM.cpp)
target_link_libraries(InfiniTAM Engine)
target_link_libraries(InfiniTAM Utils)

