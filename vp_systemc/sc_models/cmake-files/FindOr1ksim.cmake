# - Find Or1ksim
# Find the Or1ksim libraries (Or1ksim)
#
#  This module defines the following variables:
#     OR1KSIM_FOUND       - True if OR1KSIM_INCLUDE_DIR & OR1KSIM_LIBRARY are found
#     OR1KSIM_LIBRARIES   - Set when OR1KSIM_LIBRARY is found
#     OR1KSIM_INCLUDE_DIRS - Set when OR1KSIM_INCLUDE_DIR is found
#
#     OR1KSIM_INCLUDE_DIR - where to find or1ksim.h, etc.
#     OR1KSIM_LIBRARY     - the or1ksim library
#     OR1KSIM_VERSION_STRING - the version of Or1ksim found (since CMake 2.8.8)
#

#=============================================================================
# Copyright 2013 Xiao Pan  <pan@cs.uni-kl.de> ,
# Copyright 2013 Javier Moreno  <morebo@cs.uni-kl.de>
#
# Distributed under the OSI-approved BSD License (the "License");
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================

find_path(OR1KSIM_INCLUDE_DIR NAMES or1ksim.h HINTS $ENV{OR1KSIM_BASE} $ENV{OR1KSIM_HOME} $ENV{OR1KSIM}
          PATH_SUFFIXES include
          DOC "The or1ksim include directory"
)

find_library(OR1KSIM_LIBRARY NAMES sim HINTS $ENV{OR1KSIM_BASE} $ENV{OR1KSIM_HOME} $ENV{OR1KSIM}
          PATH_SUFFIXES lib lib-linux lib-linux64 lib-macosx lib-macosx64 lib-darwin
          DOC "The or1ksim library"
)


# handle the QUIETLY and REQUIRED arguments and set OR1KSIM_FOUND to TRUE if
# all listed variables are TRUE
include(${CMAKE_CURRENT_LIST_DIR}/FindPackageHandleStandardArgs.cmake)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(OR1KSIM
                                  REQUIRED_VARS OR1KSIM_LIBRARY OR1KSIM_INCLUDE_DIR
                                  VERSION_VAR OR1KSIM_VERSION_STRING)

if(OR1KSIM_FOUND)
  set( OR1KSIM_LIBRARIES ${OR1KSIM_LIBRARY} )
  set( OR1KSIM_INCLUDE_DIRS ${OR1KSIM_INCLUDE_DIR} )
endif()
