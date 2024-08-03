cmake_minimum_required(VERSION 3.10)
project(AdvancedIndoorPointCloudProcessing)

set(CMAKE_CXX_STANDARD 17)

find_package(PCL 1.10 REQUIRED)
find_package(VTK REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${VTK_INCLUDE_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(point_cloud_processor src/main.cpp src/point_cloud_processor.cpp src/point_cloud_visualizer.cpp src/mapping.cpp)

target_link_libraries(point_cloud_processor ${PCL_LIBRARIES} ${VTK_LIBRARIES})
