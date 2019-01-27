cmake_minimum_required (VERSION 2.8)
project (ecu_firmware)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_FIND_ROOT_PATH "/opt/or1k-toolchain")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_C_COMPILER "/opt/or1k-toolchain/bin/or1k-elf-gcc")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../bin)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -mnewlib -mboard=or1ksim")
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS)


add_subdirectory(src bin)
