cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

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
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(talker   src/my_publisher.cpp
                        src/pub_main.cpp)

add_executable(custom_subscriber src/custom_subscriber.cpp)

ament_target_dependencies(talker rclcpp std_msgs)                        

add_executable(listener src/my_subscriber.cpp
                        src/sub_main.cpp)

ament_target_dependencies(listener rclcpp std_msgs)
ament_target_dependencies(custom_subscriber rclcpp std_msgs)


install(TARGETS
  talker
  listener
  custom_subscriber
  DESTINATION lib/${PROJECT_NAME})


ament_package()
