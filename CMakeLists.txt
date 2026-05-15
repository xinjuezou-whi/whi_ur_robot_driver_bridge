cmake_minimum_required(VERSION 3.5)
project(whi_ur_robot_driver_bridge)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(ur_msgs REQUIRED)
find_package(ur_dashboard_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(bondcpp REQUIRED)
find_package(bond REQUIRED)
find_package(whi_interfaces REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
)

## Declare a C++ library
FILE(GLOB WHI_SDK_SRC
 "src/sdk/*.cpp"
)

add_library(${PROJECT_NAME} ${WHI_SDK_SRC})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  std_srvs
  controller_manager_msgs
  ur_msgs
  ur_dashboard_msgs
  rclcpp_action
  rclcpp_lifecycle
  bondcpp
  bond
  whi_interfaces
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  boost_system
)

#############
## Install ##
#############

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
