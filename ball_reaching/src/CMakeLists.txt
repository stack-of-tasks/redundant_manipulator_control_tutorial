
# Copyright 2011, Florent Lamiraux, CNRS
#

SET (LIBRARY_NAME ${PROJECT_NAME})

ADD_LIBRARY (${LIBRARY_NAME} SHARED
  cubic-interpolation.cc
  cubic-interpolation.hh
  cylindrical-cubic-interpolation.cc
  cylindrical-cubic-interpolation.hh
  )

PKG_CONFIG_USE_DEPENDENCY (${LIBRARY_NAME} sot-core)

DYNAMIC_GRAPH_PYTHON_MODULE ("sot/reaching" ${LIBRARY_NAME} wrap)

INSTALL (TARGETS ${LIBRARY_NAME} DESTINATION lib)
