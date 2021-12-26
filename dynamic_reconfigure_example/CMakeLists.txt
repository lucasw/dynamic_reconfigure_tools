cmake_minimum_required(VERSION 2.8.3)
project(dynamic_reconfigure_example)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  message_generation
  nodelet
  roscpp
  roslint
  std_msgs
)

roslint_cpp()
roslint_python()

generate_dynamic_reconfigure_options(
  cfg/Example.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  dynamic_reconfigure
  nodelet
  roscpp
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/example_server.cpp
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# TODO(lucasw) is this needed?
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_LIBRARIES})
endif()

add_executable(example_server_node_wrapper src/example_server_node_wrapper.cpp)
target_link_libraries(example_server_node_wrapper ${catkin_LIBRARIES})

add_executable(example_server_node src/example_server_node.cpp)
add_dependencies(example_server_node ${PROJECT_NAME}_gencfg)
target_link_libraries(example_server_node ${catkin_LIBRARIES})

# install

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS
  ${PROJECT_NAME}
  example_server_node
  example_server_node_wrapper
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/example_client.py
  scripts/example_server.py
  scripts/manual_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  nodelet_${PROJECT_NAME}.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

