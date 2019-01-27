#=======================================================================================
# Author: Xiao PAN <pan@cs.uni-kl.de> on 19/10/2018.
# Project "SystemC VP of Electoral Throttle Control System (scvp_etc)"
#======================================================================================

cmake_minimum_required (VERSION 2.8)
project (scvp_etc_demo)

message(STATUS "+---------------------------------------------------------------------------------+")
message(STATUS "|  Building Project : SystemC VP of Electoral Throttle Control System (scvp_ect)  |")
message(STATUS "|              (Read more : https://github.com/TUK-CPS/VP)                        |")
message(STATUS "+---------------------------------------------------------------------------------+")

# Set a default build type if none was specified
set(default_build_type "Release")
if(EXISTS "${CMAKE_SOURCE_DIR}/../.git")
  set(default_build_type "Debug")
endif()


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive -DSC_INCLUDE_DYNAMIC_PROCESSES")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake-files)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../bin)


find_package(SystemC REQUIRED)
find_package(SystemCAMS REQUIRED)
find_package(Or1ksim REQUIRED)

include_directories(include ${SYSTEMC_INCLUDE_DIRS}  ${SYSTEMCAMS_INCLUDE_DIRS} )

if(OR1KSIM_FOUND)
  message(STATUS "+-----------------------------------+")
	message(STATUS "|  Or1ksim ISS simulation [ ON ]    | ")
  message(STATUS "+-----------------------------------+")
  include_directories(  ${OR1KSIM_INCLUDE_DIRS})
else()
  message(STATUS "+----------------------------------------------------------------+")
  message(STATUS "|             Or1ksim ISS simulation [ OFF ]                     | ")
  message(STATUS "| (Set $OR1KSIM path to or1ksim installtion folder to enable     | ")
  message(STATUS "|               simulation with ISS.)                            | ")
  message(STATUS "+----------------------------------------------------------------+")
endif()

add_subdirectory(src bin)
