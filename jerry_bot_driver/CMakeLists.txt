cmake_minimum_required(VERSION 3.0.2)
project(jerry_bot_driver)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  joy
  roscpp
  rospy
  std_msgs
  teleop_twist_joy
  message_generation
)

add_message_files(
  FILES
  uc_states.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES jerry_bot_driver
 CATKIN_DEPENDS controller_manager hardware_interface joy roscpp rospy std_msgs teleop_twist_joy message_runtime
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/jerry_bot_hw_driver.cpp
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

add_executable(jerry_bot_hw_interface src/jerry_bot_hw_driver.cpp)
target_link_libraries(jerry_bot_hw_interface ${catkin_LIBRARIES})
add_dependencies(jerry_bot_hw_interface diff_drive_generate_messages_cpp)
