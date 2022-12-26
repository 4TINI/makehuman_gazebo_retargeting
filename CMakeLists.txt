cmake_minimum_required(VERSION 3.0.2)
project(makehuman_gazebo_retarget)

find_package(catkin REQUIRED COMPONENTS
    roscpp  
    gazebo_ros
)
find_package(gazebo REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_fsm
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS}  ${catkin_LIBRARY_DIRS} ${tiago_public_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} )

#############
## Install ##
#############

install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

add_library(ActorCollisionsPlugin SHARED src/ActorCollisionsPlugin.cc)
target_link_libraries(ActorCollisionsPlugin ${GAZEBO_LIBRARIES})