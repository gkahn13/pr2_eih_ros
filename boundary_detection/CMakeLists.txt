cmake_minimum_required(VERSION 2.8.3)
project(boundary_detection)

add_definitions("-Wno-enum-compare")

find_package(catkin REQUIRED COMPONENTS
)


find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} "/usr/local/lib")
add_definitions(${PCL_DEFINITIONS})
 
set(PCL_BUILD_TYPE Release)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(boundary_detection src/boundary_detection.cpp)
target_link_libraries(boundary_detection ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(convert_pcd src/convert_pcd.cpp)
target_link_libraries(convert_pcd ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(generate_boundary_pointcloud src/generate_boundary_pointcloud.cpp)
target_link_libraries(generate_boundary_pointcloud ${PCL_LIBRARIES} ${catkin_LIBRARIES})
