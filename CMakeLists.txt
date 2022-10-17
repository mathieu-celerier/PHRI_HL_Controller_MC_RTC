cmake_minimum_required(VERSION 3.1)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME PHRI_HLController)
set(PROJECT_DESCRIPTION "humanLikeController")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)

# Check if the project is built inside mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

# To use ros
find_package(catkin REQUIRED COMPONENTS roscpp)
catkin_package(CATKIN_DEPENDS geometry_msgs roscpp roslib std_msgs hl_phri_exp)
message("=========== Catkin include directories : ${catkin_INCLUDE_DIRS}")
include_directories(include ${catkin_INCLUDE_DIRS} $ENV{HOME}/.local/include)
link_directories(${catkin_LIBRARY_DIRS} $ENV{HOME}/.local/lib)

add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/humanLikeController.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/humanLikeController.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/humanLikeController.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")

