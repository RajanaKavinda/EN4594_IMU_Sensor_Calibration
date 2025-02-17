cmake_minimum_required(VERSION 3.8)
project(zumo_imu_kf)

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
find_package(zumo_msgs REQUIRED)
find_package(zumo_defs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

#find_package(Boost REQUIRED COMPONENTS system thread)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

include_directories(include)

# Add executable
add_executable(zumo_imu_kf_node		 	src/zumo_imu_kf.cpp
																		src/main.cpp)

ament_target_dependencies(zumo_imu_kf_node      rclcpp 
                                                zumo_msgs
                                                zumo_defs
                                                tf2
                                                tf2_ros
                                                Eigen3)

                                              
                                                
install(TARGETS   
zumo_imu_kf_node
DESTINATION lib/${PROJECT_NAME})
                                                
ament_package()
