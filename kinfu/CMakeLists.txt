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
)
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
set(PCL_BUILD_TYPE Release)

catkin_package(
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

add_executable(kinfu src/kinfu.cpp)

target_link_libraries(kinfu ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})

