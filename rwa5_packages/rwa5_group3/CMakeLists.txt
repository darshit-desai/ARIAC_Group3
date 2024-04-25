cmake_minimum_required(VERSION 3.8)
project(rwa5_group3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Boost REQUIRED COMPONENTS system serialization)
find_package(rosidl_default_generators REQUIRED)
# include folder which has all the header files
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

# create a variable for all dependencies
set(MOVEIT_DEMO_INCLUDE_DEPENDS
  rclcpp 
  std_msgs
  ariac_msgs
  tf2_ros
  geometry_msgs
  tf2
  orocos_kdl
  tf2_ros
  tf2_geometry_msgs
  moveit_msgs
  geometric_shapes
  moveit_ros_planning_interface
  robot_commander_msgs
)

# find all dependencies
foreach(dependency IN ITEMS ${MOVEIT_DEMO_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

set(srv_files
  "srv/AdvancedCamera.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES
  ariac_msgs
)


#-----------------------------
# C++
#-----------------------------
add_executable(moveit_demo_cpp_exe src/floor_robot_cpp_only.cpp src/main_cpp_only.cpp)
ament_target_dependencies(moveit_demo_cpp_exe ${MOVEIT_DEMO_INCLUDE_DEPENDS})
target_link_libraries(moveit_demo_cpp_exe ${Boost_LIBRARIES})

rosidl_target_interfaces(moveit_demo_cpp_exe ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS moveit_demo_cpp_exe
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#-----------------------------
# Python
#-----------------------------
# Install Python modules
ament_python_install_package(${PROJECT_NAME})

#-----------------------------
# Installation
#-----------------------------

# Install Python executables
install(PROGRAMS
  script/floor_robot_demo.py
  script/yolo_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
moveit_demo_cpp_exe
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION share/${PROJECT_NAME}/
)



install(DIRECTORY
  model
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  meshes
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
