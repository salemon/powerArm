cmake_minimum_required(VERSION 3.8)
project(powerarm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)

# find_package(tf2 REQUIRED)
# find_package(tf2_eigen REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(tf2_ros REQUIRED)

# add_executable(powerarm_control_joint_space src/powerarm_control_joint_space.cpp)
# target_include_directories(powerarm_control_joint_space PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# target_link_libraries(powerarm_control_joint_space yaml-cpp)

# target_compile_features(powerarm_control_joint_space PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ament_target_dependencies(
#   powerarm_control_joint_space
#   "moveit_ros_planning_interface"
#   "moveit_visual_tools"
#   "rclcpp"
# )

# install(TARGETS powerarm_control_joint_space
#   DESTINATION lib/${PROJECT_NAME})

add_executable(powerarm_control_IK src/powerarm_control_IK.cpp)
target_include_directories(powerarm_control_IK PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(powerarm_control_IK yaml-cpp)

target_compile_features(powerarm_control_IK PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  powerarm_control_IK
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)

install(TARGETS powerarm_control_IK
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
