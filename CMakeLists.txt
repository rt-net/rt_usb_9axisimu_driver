cmake_minimum_required(VERSION 2.8.3)
project(rt_usb_9axisimu_driver)

add_compile_options(-std=c++11)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  roslint
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
  # DEPENDS system_lib
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ executable
add_executable(rt_usb_9axisimu_driver src/rt_usb_9axisimu_driver_node.cpp src/rt_usb_9axisimu_driver.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(rt_usb_9axisimu_driver ${catkin_LIBRARIES})

# Add cmake target dependencies of the library
# add_dependencies(rt_usb_9axisimu_driver rt_usb_9axisimu_driver_generate_messages_cpp)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS rt_usb_9axisimu_driver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rt_usb_9axisimu_driver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

roslint_cpp()
roslint_add_test()
