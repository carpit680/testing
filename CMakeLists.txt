cmake_minimum_required(VERSION 2.8.3)
project(testing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy_tutorials
)

add_definitions(-std=c++11 -Wall -g)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs rospy_tutorials
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare C++ library
add_library(testing
	src/talker.cpp
)

target_link_libraries(testing ${catkin_LIBRARIES})

# add_dependencies(talker_node testing)

add_executable(talker_node nodes/talker_node.cpp)
target_link_libraries(talker_node ${catkin_LIBRARIES} testing)

install(TARGETS testing
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(testing-test launch/gtest.test tests/tests_1.cpp)
  target_link_libraries(testing-test ${catkin_LIBRARIES} testing)
endif()