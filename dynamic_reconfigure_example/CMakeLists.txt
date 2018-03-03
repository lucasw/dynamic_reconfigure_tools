cmake_minimum_required(VERSION 2.8.3)
project(dynamic_reconfigure_example)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  nodelet
  roscpp
  roslint
  std_msgs
)

roslint_cpp()

generate_dynamic_reconfigure_options(
  cfg/Example.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamic_reconfigure_example
  CATKIN_DEPENDS nodelet roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(dynamic_reconfigure_example
  src/example_server.cpp
)
add_dependencies(dynamic_reconfigure_example ${PROJECT_NAME}_gencfg)
target_link_libraries(dynamic_reconfigure_example ${catkin_LIBRARIES})

# TODO(lucasw) is this needed?
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(dynamic_reconfigure_example ${catkin_EXPORTED_LIBRARIES})
endif()

add_executable(example_server_node_wrapper src/example_server_node_wrapper.cpp)
target_link_libraries(example_server_node_wrapper ${catkin_LIBRARIES})

add_executable(example_server_node src/example_server_node.cpp)
target_link_libraries(example_server_node ${catkin_LIBRARIES})

# TODO(lucasw) install
