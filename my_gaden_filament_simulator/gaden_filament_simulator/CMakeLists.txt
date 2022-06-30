
cmake_minimum_required(VERSION 2.8.3)
project(gaden_filament_simulator CXX)

set(CMAKE_CXX_FLAGS "-lboost_iostreams -std=c++11 -fopenmp ${CMAKE_CXX_FLAGS}")
find_package(catkin REQUIRED COMPONENTS
	roscpp
	visualization_msgs
	std_msgs
	nav_msgs
	pcl_ros
  message_generation

)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams)

add_message_files(
  FILES
  coorMsg.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
    DEPENDS roscpp visualization_msgs std_msgs nav_msgs Boost 
)

# include_directories(include
# ${catkin_INCLUDE_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS} $(PCL_INCLUDE_DIRS) ${OpenCV_INCLUDE_DIRS})
MESSAGE("PCL ARE: ${PCL_INCLUDE_DIRS}")



FILE(GLOB_RECURSE MYFILES_CPP "src/*.cpp")

add_executable(single_filament_simulator ${MYFILES_CPP})
add_executable(filament_simulator ${MYFILES_CPP})

target_link_libraries(single_filament_simulator
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )
 
target_link_libraries(filament_simulator
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )
