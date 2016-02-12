# Try to find ACADO (https://github.com/acado/acado) with cmake
#
# This script assumes ACADO Toolkit to be installed according to the Linux
# installation instructions and having called
# $ sudo make install
# after make.
# 
# It defines the following variables:
#  ACADOToolkit_FOUND 
#  ACADOToolkit_INCLUDE_DIRS 
#  ACADOToolkit_LIBRARIES 
#  ACADOToolkit_LIBRARY_DIRS 
#
# Paul Manns, Jan 2016
#

set( ACADOToolkit_FOUND OFF )

if( UNIX )
   
   find_path(INCA
             NAMES "acado_optimal_control.hpp"
             HINTS /usr/include/acado
                   /usr/local/include/acado)

   set(ACADOToolkit_INCLUDE_DIRS "${INCA}"
                                 "${INCA}/bindings"
                                 "${ACADOToolkit_INCLUDE_DIRS}")


   find_path(ACADOToolkit_LIBRARY_DIR
             NAMES "libacado_lib.so" "libacado_toolkit_s.so"
             HINTS
             /usr/lib
             /usr/local/lib)

   set(ACADOToolkit_LIBRARY_DIRS ${ACADOToolkit_LIBRARY_DIR}
                                 ${ACADOToolkit_LIBRARY_DIRS})

   find_library(ACADOToolkit_LIBRARIES 
             NAMES acado_lib acado_toolkit_s
             HINTS ${ACADOToolkit_LIBRARY_DIRS})


   if (INCA AND ACADOToolkit_LIBRARY_DIR AND ACADOToolkit_LIBRARIES)
     set(ACADOToolkit_FOUND ON)
     message(STATUS "Found ACADOToolkit")
   else()
     message(WARNING "ACADOToolkit not found on your system.")
   endif()
else(UNIX)
   message(WARNING "FindACADOToolkit.cmake currently only works on Linux.")
endif(UNIX)

