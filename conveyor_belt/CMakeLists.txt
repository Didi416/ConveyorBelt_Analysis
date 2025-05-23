cmake_minimum_required(VERSION 3.8)
project(conveyor_belt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(conveyor_belt_msgs REQUIRED)
# find_package(PCL REQUIRED)
# find_package(pcl_ros REQUIRED)
# find_package(pcl_conversions REQUIRED)

# add_executable(filter src/filter.cpp)
# ament_target_dependencies(filter rclcpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros tf2_sensor_msgs tf2_geometry_msgs)
# add_executable(loadProfile src/loadProfile.cpp)
# ament_target_dependencies(loadProfile rclcpp std_msgs sensor_msgs geometry_msgs)
add_executable(spawnObjects src/spawnObjects.cpp)
ament_target_dependencies(spawnObjects rclcpp std_msgs gazebo_msgs)
add_executable(despawnObjects src/despawnObjects.cpp)
ament_target_dependencies(despawnObjects rclcpp std_msgs gazebo_msgs conveyor_belt_msgs)
add_executable(service src/service.cpp)
ament_target_dependencies(service rclcpp gazebo_msgs conveyor_belt_msgs)

install(TARGETS
  spawnObjects
  despawnObjects
  service
  DESTINATION lib/conveyor_belt)

# Install Python scripts
# ament_python_install_package(scripts) 
install(PROGRAMS 
  scripts/loadProfile.py
  DESTINATION lib/conveyor_belt
)

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

install(DIRECTORY launch models worlds urdf rviz
  DESTINATION share/conveyor_belt
)

ament_package()
