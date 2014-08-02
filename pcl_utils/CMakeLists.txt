cmake_minimum_required(VERSION 2.8.3)
project(pcl_utils)

add_definitions("-Wno-enum-compare")

find_package(catkin REQUIRED COMPONENTS
)

find_package(PCL 1.7 REQUIRED)

set(PCL_INCLUDE_DIRS "/usr/local/include/pcl-1.7" "/usr/include/eigen3" "/usr/include/ni" "/usr/include/vtk-5.8" "/usr/include/qhull")

include_directories(${PCL_INCLUDE_DIRS} "/usr/local/lib")
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
message(${PCL_LIBRARY_DIRS})
message(${PCL_LIBRARIES})
message(${PCL_INCLUDE_DIRS})

set(PCL_BUILD_TYPE Release)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(Eigen)

include_directories(
    ${Eigen_INCLUDE_DIRS}
    "${PROJECT_SOURCE_DIR}/include"
    )

add_executable(boundary_detection src/boundary_detection.cpp)
target_link_libraries(boundary_detection ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(convert_pcd src/convert_pcd.cpp)
target_link_libraries(convert_pcd ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(generate_boundary_pointcloud src/generate_boundary_pointcloud.cpp)
target_link_libraries(generate_boundary_pointcloud ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(plane_recognition src/plane_recognition.cpp)
target_link_libraries(plane_recognition ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(find_empty_voxels src/find_empty_voxels.cpp)
target_link_libraries(find_empty_voxels ${PCL_LIBRARIES} ${catkin_LIBRARIES})
