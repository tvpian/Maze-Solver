cmake_minimum_required(VERSION 3.8)
project(group1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mage_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)


if(ament_cmake_FOUND)
  include_directories(${ament_cmake_INCLUDE_DIRS})
endif()

if(rclcpp_FOUND)
  include_directories(${rclcpp_INCLUDE_DIRS})
endif()

if(mage_msgs_FOUND)
  include_directories(${mage_msgs_INCLUDE_DIRS})
endif()

if(geometry_msgs_FOUND)
  include_directories(${geometry_msgs_INCLUDE_DIRS})
endif()

if(std_msgs_FOUND)
  include_directories(${std_msgs_INCLUDE_DIRS})
endif()

if(tf2_ros_FOUND)
  include_directories(${tf2_ros_INCLUDE_DIRS})
endif()

if(tf2_FOUND)
  include_directories(${tf2_INCLUDE_DIRS})
endif()

if(tf2_geometry_msgs_FOUND)
  include_directories(${tf2_geometry_msgs_INCLUDE_DIRS})
endif()

if(nav_msgs_FOUND)
  include_directories(${nav_msgs_INCLUDE_DIRS})
endif()

if(tf2_FOUND)
  include_directories(${tf2_INCLUDE_DIRS})
endif()


# if(ros2_aruco_FOUND)
#   include_directories(${ros2_aruco_INCLUDE_DIRS})
# endif()

# if(ros2_aruco_interfaces_FOUND)
#   include_directories(${ros2_aruco_interfaces_INCLUDE_DIRS})
# endif()



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)


# group1 tb3_aruco_broadcaster node
add_executable(tb3_aruco_broadcaster_node src/tb3_aruco_detector_bcaster.cpp src/utils.cpp)
ament_target_dependencies(tb3_aruco_broadcaster_node rclcpp mage_msgs geometry_msgs std_msgs tf2_ros tf2 tf2_geometry_msgs ros2_aruco_interfaces nav_msgs tf2)

# group1 tb3_aruco_listener node
add_executable(tb3_aruco_listener_node src/listener_demo_dcomp.cpp src/utils.cpp)
ament_target_dependencies(tb3_aruco_listener_node rclcpp mage_msgs geometry_msgs std_msgs tf2_ros tf2 tf2_geometry_msgs ros2_aruco_interfaces nav_msgs tf2)

# group1 tb3_parts_broadcaster node
add_executable(tb3_parts_broadcaster_node src/tb3_parts_broadcaster.cpp src/utils.cpp)
ament_target_dependencies(tb3_parts_broadcaster_node rclcpp mage_msgs geometry_msgs std_msgs tf2_ros tf2 tf2_geometry_msgs ros2_aruco_interfaces nav_msgs tf2)

# group1 tb3_parts_listener node
add_executable(tb3_parts_listener_node src/tb3_parts_listener.cpp src/utils.cpp)
ament_target_dependencies(tb3_parts_listener_node rclcpp mage_msgs geometry_msgs std_msgs tf2_ros tf2 tf2_geometry_msgs ros2_aruco_interfaces nav_msgs tf2)


install(TARGETS
  tb3_aruco_broadcaster_node
  tb3_aruco_listener_node
  tb3_parts_broadcaster_node
  tb3_parts_listener_node
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY include config launch 
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
