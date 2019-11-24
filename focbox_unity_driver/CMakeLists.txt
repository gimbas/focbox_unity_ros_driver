cmake_minimum_required(VERSION 2.8.3)
project(focbox_unity_driver)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  pluginlib
  roscpp
  std_msgs
  focbox_unity_msgs
  serial
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS nodelet pluginlib roscpp std_msgs focbox_unity_msgs serial
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# node executable
add_executable(focbox_unity_driver_node src/focbox_unity_driver_node.cpp
                                        src/focbox_unity_driver.cpp
                                        src/focbox_unity_interface.cpp
                                        src/focbox_unity_packet.cpp
                                        src/focbox_unity_packer.cpp)
add_dependencies(focbox_unity_driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(focbox_unity_driver_node
  ${catkin_LIBRARIES}
)

# nodelet library
add_library(focbox_unity_driver_nodelet src/focbox_unity_driver_nodelet.cpp
                                src/focbox_unity_driver.cpp
                                src/focbox_unity_interface.cpp
                                src/focbox_unity_packet.cpp
                                src/focbox_unity_packer.cpp)
add_dependencies(focbox_unity_driver_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(focbox_unity_driver_nodelet
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS focbox_unity_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS focbox_unity_driver_nodelet
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES focbox_unity_driver_nodelet.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

#############
## Testing ##
#############

# TODO