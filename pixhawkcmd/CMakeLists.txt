cmake_minimum_required(VERSION 2.8.3)
project(pixhawkcmd)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  rospy
  sensor_msgs
  std_msgs
  mavros_msgs
  mavros
  tf
)

catkin_package(
  DEPENDS eigen3
)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
endif()

# include_directories(include)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


# Declare a cpp executable

#add_executable(listener_px4 src/streamPX4.cpp)
#target_link_libraries(listener_px4 ${catkin_LIBRARIES})
#add_executable(all_px4 src/PX4all.cpp)
#target_link_libraries(all_px4 ${catkin_LIBRARIES})
#add_dependencies()
add_executable(pixhawk_send src/pixhawk_sendV2.cpp)
target_link_libraries(pixhawk_send ${catkin_LIBRARIES})

