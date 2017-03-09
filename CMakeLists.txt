cmake_minimum_required(VERSION 2.8.3)
project(pal_fsp_gui)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  rviz
  pal_footstep_planner_msgs
  interactive_markers
  pal_robot_tools
)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -fPIC")

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pal_motion_builder
#  CATKIN_DEPENDS play_motion_msgs
#  DEPENDS system_lib
)

qt4_wrap_cpp(pal_fsp_gui_MOCS
  include/${PROJECT_NAME}/pal_fsp_gui.h
  include/${PROJECT_NAME}/fsp_panel.h)
qt4_wrap_ui(pal_fsp_gui_UIS_H
  resources/pal_fsp_gui.ui)

###########
## Build ##
###########
include_directories(include)
include_directories(SYSTEM ${CMAKE_CURRENT_BINARY_DIR} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/pal_fsp_gui.cpp
  src/fsp_panel.cpp
  ${pal_fsp_gui_MOCS}
  ${pal_fsp_gui_UIS_H})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_QTCORE_LIBRARY}
  ${QT_QTGUI_LIBRARY})

find_package(class_loader REQUIRED)
class_loader_hide_library_symbols(${PROJECT_NAME})

#############
## Install ##
#############
install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
