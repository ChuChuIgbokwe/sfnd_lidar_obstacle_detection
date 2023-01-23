cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#add_definitions(-std=c++11)
add_definitions(-std=c++14)
#Add the installation prefix of "PCL" to CMAKE_PREFIX_PATH or set "PCL_DIR"
#  to a directory containing one of the above files.  If "PCL" provides a
#  separate development package or SDK, be sure it has been installed.
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

set(CMAKE_MODULE_PATH "/usr/include/pcl-1.13")
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#add_definitions(-std=c++14)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp)
target_link_libraries (environment ${PCL_LIBRARIES})

add_subdirectory(src/quiz/ransac)
add_subdirectory(src/quiz/cluster)



