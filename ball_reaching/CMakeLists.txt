
# Copyright 2011, Florent Lamiraux, CNRS
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

FINDPYTHON ()

SET(PROJECT_NAME sot-ball-reaching)
SET(PROJECT_DESCRIPTION "sot-reaching")
SET(PROJECT_URL "http://github.com/laas/sot-reaching")

SET(CUSTOM_HEADER_DIR sot/reaching)
SET(${PROJECT_NAME}_HEADERS
)

SET(CXX_DISABLE_WERROR 1)
SETUP_PROJECT()

# Trigger dependency to dynamic-graph-python
ADD_REQUIRED_DEPENDENCY("sot-core >= 2.5")

# Add dependency toward dynamic-graph-corba library in pkg-config file.
PKG_CONFIG_APPEND_LIBS("sot-reaching")

ADD_SUBDIRECTORY(src)

INSTALL(FILES
  reach_ball.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/reaching
)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()

# Ros part
ROSBUILD_FIND_ROS_PACKAGE(actionlib_msgs)
INCLUDE(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
GENACTION()
ROSBUILD_INIT()

#set the default path for built executables to the "bin" directory
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

ROSBUILD_GENMSG()
