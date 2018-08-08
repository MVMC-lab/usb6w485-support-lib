cmake_minimum_required(VERSION 2.6)
project(USB6W485lib C CXX)

# The VERSION number
set (USB6W485lib_VERSION_MAJOR 0)
set (USB6W485lib_VERSION_MINOR 1)
set (USB6W485lib_VERSION_PATCH 0)

# Must use GNUInstallDirs to install libraries into correct
# locations on all platforms.
include(GNUInstallDirs)

# Configure a header file to pass cmake setting to the source file
configure_file(
    "${PROJECT_SOURCE_DIR}/cmake/usb6w485lib_config.h.in"
    "${PROJECT_BINARY_DIR}/usb6w485lib_config.h"
)

if (USE_DEBUG_MSG)
    # TODO add MACRO to impeliment debug message report
ENDIF (USE_DEBUG_MSG)

enable_testing()
add_subdirectory(src)

install (FILES "${CMAKE_BINARY_DIR}/usb6w485lib_config.h"
DESTINATION "${CMAKE_BINARY_DIR}/include"
)
