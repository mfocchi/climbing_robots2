######################
# set up the project #
######################
cmake_minimum_required(VERSION 2.8.3)
project(climbingrobot_description)


# required to use std::shared_ptr in code AND to link the DG plugins
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--no-as-needed")


############################
# Search for dependencies. #
############################

# find dependencies using catkin

find_package(catkin REQUIRED std_msgs geometry_msgs message_generation
  COMPONENTS
)


add_message_files(
  DIRECTORY
  msg
  FILES
  RopeCommand.msg
  PropellerCommand.msg
  RopeTelemetry.msg
  AlpineBodyTelemetry.msg
)

add_service_files(
  DIRECTORY
  srv
  FILES
  AlpineBodyCommand.srv
  RopeControlMode.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)



catkin_package(
  CATKIN_DEPENDS geometry_msgs std_msgs 
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY gazebo/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/gazebo
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  PATTERN ".svn" EXCLUDE)


install(DIRECTORY meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
  PATTERN ".svn" EXCLUDE)


install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY media/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media
  PATTERN ".svn" EXCLUDE)


install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  PATTERN ".svn" EXCLUDE)




catkin_install_python(PROGRAMS scripts/go0
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
