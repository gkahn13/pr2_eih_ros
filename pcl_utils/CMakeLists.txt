cmake_minimum_required(VERSION 2.8.3)
project(pcl_utils)

add_definitions("-Wno-enum-compare")

find_package(catkin REQUIRED COMPONENTS
    roscpp
)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
set(PCL_LIBRARIES ${PCL_LIBRARIES} "pcl_common")

set(PCL_BUILD_TYPE Release)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories("${PROJECT_SOURCE_DIR}/include"
  )

link_directories(#${Boost_LIBRARY_DIRS}
  )


add_executable(boundary_detection src/boundary_detection.cpp)
target_link_libraries(boundary_detection ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(convert_pcd src/convert_pcd.cpp)
target_link_libraries(convert_pcd ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(generate_boundary_pointcloud src/generate_boundary_pointcloud.cpp)
target_link_libraries(generate_boundary_pointcloud ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(plane_recognition src/plane_recognition.cpp)
#target_link_libraries(plane_recognition ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(find_empty_voxels src/find_empty_voxels.cpp src/plane_recognition.cpp)
target_link_libraries(find_empty_voxels ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(cluster_extraction src/cluster_extraction.cpp)
target_link_libraries(cluster_extraction ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(tsdf_converter src/tsdf_converter.cpp src/pointcloud_voxel_grid.cpp)
target_link_libraries(tsdf_converter ${PCL_LIBRARIES} ${catkin_LIBRARIES})
