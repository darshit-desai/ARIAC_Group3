# cmake_minimum_required(VERSION 3.8)
# project(rwa4_group3)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # Create a variable for all dependencies
# set(FRAME_DEMO_INCLUDE_DEPENDS
#   ament_cmake
#   rclcpp
#   std_msgs
#   std_srvs
#   ariac_msgs 
#   geometry_msgs
#   tf2
#   orocos_kdl
#   tf2_ros
#   tf2_geometry_msgs
# )

# find_package(Boost REQUIRED COMPONENTS system serialization)

# # Find all dependencies
# foreach(dependency IN ITEMS ${FRAME_DEMO_INCLUDE_DEPENDS})
#   find_package(${dependency} REQUIRED)
# endforeach()

# include_directories(include)
# include_directories(${Boost_INCLUDE_DIRS})

# add_executable(competition_demo src/competition_demo.cpp src/utils_demo.cpp)
# ament_target_dependencies(competition_demo ${FRAME_DEMO_INCLUDE_DEPENDS})
# target_link_libraries(competition_demo ${Boost_LIBRARIES})

# install(TARGETS
#   competition_demo
#   DESTINATION lib/${PROJECT_NAME}
# )

# # Install directories in the correct destination
# install(DIRECTORY
#   launch
#   config
#   DESTINATION share/${PROJECT_NAME}
# )

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()

cmake_minimum_required(VERSION 3.8)
project(rwa4_group3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Create a variable for all dependencies
set(FRAME_DEMO_INCLUDE_DEPENDS
  ament_cmake
  ament_cmake_python
  rclcpp
  std_msgs
  std_srvs
  ariac_msgs 
  geometry_msgs
  tf2
  orocos_kdl
  tf2_ros
  tf2_geometry_msgs
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ariac_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system serialization)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${ariac_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(competition_demo src/competition_demo.cpp src/utils_demo.cpp)
ament_target_dependencies(competition_demo ${FRAME_DEMO_INCLUDE_DEPENDS})
target_link_libraries(competition_demo
  ${ament_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${std_srvs_LIBRARIES}
  ${ariac_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  Boost::system
  Boost::serialization
)

install(TARGETS
  competition_demo
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories in the correct destination
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

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

ament_package()
