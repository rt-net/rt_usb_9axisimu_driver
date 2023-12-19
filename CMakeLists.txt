cmake_minimum_required(VERSION 3.5)
project(rt_usb_9axisimu_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

add_library(rt_usb_9axisimu_driver_component SHARED
  src/rt_usb_9axisimu_driver_component.cpp
  src/rt_usb_9axisimu_driver.cpp)
target_compile_definitions(rt_usb_9axisimu_driver_component
  PRIVATE "RT_USB_9AXISIMU_DRIVER_BUILDING_DLL")
ament_target_dependencies(rt_usb_9axisimu_driver_component
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  std_srvs)
rclcpp_components_register_nodes(rt_usb_9axisimu_driver_component "rt_usb_9axisimu_driver::Driver")

add_executable(rt_usb_9axisimu_driver src/rt_usb_9axisimu_driver_node.cpp)
target_link_libraries(rt_usb_9axisimu_driver rt_usb_9axisimu_driver_component)
ament_target_dependencies(rt_usb_9axisimu_driver
  rclcpp)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(rclcpp_lifecycle)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(std_srvs)

ament_export_include_directories(include)
ament_export_libraries(
  rt_usb_9axisimu_driver_component)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  rt_usb_9axisimu_driver_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(TARGETS
  rt_usb_9axisimu_driver
  DESTINATION lib/${PROJECT_NAME})

# launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  ament_lint_auto_find_test_dependencies()
  Set(FETCHCONTENT_QUIET FALSE)
  include(FetchContent)
  FetchContent_Declare(
    fakeit
    GIT_REPOSITORY https://github.com/eranpeer/FakeIt
    GIT_TAG 2.4.0
    GIT_PROGRESS TRUE)
  FetchContent_MakeAvailable(fakeit)

  find_package(ament_cmake_gtest)
  ament_add_gtest(test_driver test/test_driver.cpp src/rt_usb_9axisimu_driver.cpp)
  target_include_directories(test_driver PRIVATE ${fakeit_SOURCE_DIR}/single_header/gtest)
  ament_target_dependencies(test_driver
    rclcpp
    sensor_msgs
  )
endif()

ament_package()