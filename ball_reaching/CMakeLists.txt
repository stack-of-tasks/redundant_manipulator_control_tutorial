
# Copyright 2011, Florent Lamiraux, CNRS
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/python.cmake)

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
  ball_reaching.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/reaching
)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
