cmake_minimum_required(VERSION 3.8)
project(zumo_serial)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(zumo_msgs REQUIRED)
find_package(zumo_defs REQUIRED)
# Add Boost components you need
find_package(Boost REQUIRED COMPONENTS thread system)

include_directories(include ${Boost_INCLUDE_DIRS})

# Add executable
add_executable(zumo_serial_node   src/zumo_serial_node.cpp
                                  src/main.cpp)

target_include_directories(zumo_serial_node PUBLIC ${SERIAL_INCLUDE_DIRS}) 

# Specify libraries to link a library or executable target against
target_link_libraries(zumo_serial_node   serial ${Boost_LIBRARIES})
target_compile_options(zumo_serial_node  PUBLIC -DSERIAL_ENABLED)

ament_target_dependencies(zumo_serial_node  rclcpp std_msgs zumo_msgs zumo_defs)            


install(TARGETS
  zumo_serial_node  
  DESTINATION lib/${PROJECT_NAME})

ament_package()
