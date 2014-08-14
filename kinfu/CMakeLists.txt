cmake_minimum_required(VERSION 2.8.3)
project(kinfu)

add_definitions("-Wno-enum-compare")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl
  pcl_ros
  roscpp
  sensor_msgs
  tf
  tf_conversions
  pcl_utils
)
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
set(PCL_LIBRARIES ${PCL_LIBRARIES} "pcl_common")

set(PCL_BUILD_TYPE Release)


catkin_package(
    #CATKIN_DEPENDS pcl_utils
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############



find_package(OpenRAVE REQUIRED)
find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT COMPONENTS iostreams thread filesystem system program_options graph)
find_package(Eigen)
find_package(CUDA)


# find_package(OPENNI)

# set(PCL_INCLUDE_DIR "/usr/local/include/pcl-1.7")
# set(PCL_LIBRARIES_DIR "/usr/local/lib")
# #set(OPENNI_INCLUDE_DIRS "/usr/include/ni")
# #set(OPENNI_LIBRARY_DIRS "/usr/lib")


# #set(OPENNI_LIBRARIES "${OPENNI_LIBRARIES};Xn")

include_directories(${OpenRAVE_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} ${PYTHON_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  )

link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS}
  )

message(STATUS "PCL UTILS VARIABLES")
message(STATUS ${pcl_utils_SOURCE_DIR})
message(STATUS ${pcl_utils_LIBRARIES})

#add_executable(kinfu src/kinfu.cpp ${pcl_utils_SOURCE_DIR}/src/occluded_region_finder.cpp ${pcl_utils_SOURCE_DIR}/src/cluster_projection.cpp
#    ${pcl_utils_SOURCE_DIR}/src/pointcloud_voxel_grid.cpp ${pcl_utils_SOURCE_DIR}/src/plane_recognition.cpp ${pcl_utils_SOURCE_DIR}/src/cluster_extraction.cpp ${pcl_utils_SOURCE_DIR}/src/tsdf_converter.cpp
#)
#target_link_libraries(kinfu ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
#add_dependencies(kinfu pcl_utils_generate_messages_cpp
#pcl_utils
#)

