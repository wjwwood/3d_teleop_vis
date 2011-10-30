cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

FIND_PACKAGE(VTK)
INCLUDE(${VTK_USE_FILE}) # include UseVTK.cmake
 
FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})  # include UseQt4.cmake

# support for out-of-source build
INCLUDE_DIRECTORIES(
  ${CMAKE_CURRENT_BINARY_DIR} #this is where ui_SimpleViewUI.h is generated
)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# Set sources and resources
SET(SRCS src/octree_vis_widget.cpp)
SET(UIS ui/octree_vis_widget.ui)
SET(HEADERS include/octree_vis_widget/octree_vis_widget.h)

QT4_WRAP_UI(UI_SRCS ${UIS})
QT4_WRAP_CPP(MOC_SRCS ${HEADERS})

SOURCE_GROUP("Resources" FILES
  ${UIS}
)
 
SOURCE_GROUP("Generated" FILES
  ${UI_SRCS}
  ${MOC_SRCS}
)

rosbuild_add_library(${PROJECT_NAME} ${SRCS} ${UI_SRCS} ${MOC_SRCS})
target_link_libraries(${PROJECT_NAME} QVTK)


rosbuild_add_executable(octree_vis_widget_node src/octree_vis_widget_node.cpp)
target_link_libraries(octree_vis_widget_node ${PROJECT_NAME})
