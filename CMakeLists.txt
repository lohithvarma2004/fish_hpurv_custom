cmake_minimum_required(VERSION 3.8)
project(fish_hpurv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(xacro REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Add executable for joint_publisher
add_executable(joint_publisher scripts/joint_publisher.cpp)
ament_target_dependencies(joint_publisher
  rclcpp
  trajectory_msgs
)
install(TARGETS joint_publisher
  DESTINATION lib/${PROJECT_NAME}
)

#Add executable for hpurv_dynamics_node
 add_executable(hpurv_dynamics_node scripts/hpurv_dynamics_node.cpp)
 ament_target_dependencies(hpurv_dynamics_node
   rclcpp
   trajectory_msgs
   sensor_msgs
   geometry_msgs
 )
 target_include_directories(hpurv_dynamics_node PUBLIC
   ${EIGEN3_INCLUDE_DIRS}
 )
 target_link_libraries(hpurv_dynamics_node
   Eigen3::Eigen
 )
 install(TARGETS hpurv_dynamics_node
   DESTINATION lib/${PROJECT_NAME}
 )

# Add shared library for Gazebo plugin
add_executable(force_dynamic_plugin scripts/force_dynamic_plugin.cpp)
target_include_directories(force_dynamic_plugin PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(force_dynamic_plugin
  ${GAZEBO_LIBRARIES}
  Eigen3::Eigen
)
ament_target_dependencies(force_dynamic_plugin
  rclcpp
  gazebo_ros
  trajectory_msgs
  sensor_msgs
  geometry_msgs
)
install(TARGETS force_dynamic_plugin
  DESTINATION lib/${PROJECT_NAME}
)
add_library(CaudalForcePlugin SHARED scripts/CaudalForcePlugin.cpp)

target_include_directories(CaudalForcePlugin PUBLIC
  ${GAZEBO_INCLUDE_DIRS}
)

ament_target_dependencies(CaudalForcePlugin
  rclcpp
  std_msgs
  gazebo_ros
)

target_link_libraries(CaudalForcePlugin
  ${GAZEBO_LIBRARIES}
  ignition-math6::ignition-math6
)

install(TARGETS CaudalForcePlugin
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)


# Install directories
install(DIRECTORY launch urdf config meshes worlds models
  DESTINATION share/${PROJECT_NAME}
)

# Install xacro files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.xacro"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
