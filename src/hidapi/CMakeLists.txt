#Define library. Only source file here!
project(HidAPI LANGUAGES CXX)
cmake_minimum_required (VERSION 2.6.2)

# does this system provide specific functions?
include (${CMAKE_ROOT}/Modules/CheckFunctionExists.cmake)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

if(WIN32)
    message("Usb HIDAPI Library with WIN32 platform")
    find_package(WinHID REQUIRED)
    add_library(${PROJECT_NAME}
        src/hid_win32.c
    )
    target_link_libraries(${PROJECT_NAME} ${WINHID_LIBRARIES})

    if(MINGW)
    # TODO: Add mingw32 support
    endif()

endif(WIN32)

if(UNIX)
    find_package(catkin REQUIRED)
    catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include
        DEPENDS rt pthread
    )
    find_package(LibUSB)
    if( LIBUSB_FOUND)
        message("Usb HIDAPI Library with Linux libusb platform")
        add_library(${PROJECT_NAME}
            src/hid_linux_libusb.c
        )
        target_link_libraries(${PROJECT_NAME} ${LIBUSB_LIBRARY})
        message("Usb HIDAPI LINK: " ${LIBUSB_LIBRARY})
        include_directories(${LIBUSB_INCLUDE_DIR})
    else()
        message("Usb HIDAPI Library with Linux platform")
        add_library(${PROJECT_NAME}
            src/hid_linux.c
        )
    endif()
endif(UNIX)

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# If we have compiler requirements for this library, list them
# here
target_compile_features(${PROJECT_NAME}
    PUBLIC cxx_auto_type
    PRIVATE cxx_variadic_templates)

if(UNIX)
    # 'make install' to the correct locations (provided by GNUInstallDirs).
    install(TARGETS ${PROJECT_NAME}
    EXPORT "${PROJECT_NAME}Config"
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})  # This is for Linux with ROS

    install(DIRECTORY include/
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE
    )
else(UNIX)
    # 'make install' to the correct locations (provided by GNUInstallDirs).
    install(TARGETS ${PROJECT_NAME}
    EXPORT "${PROJECT_NAME}Config"
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows

    install(DIRECTORY include/ 
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )
endif(UNIX)

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT "${PROJECT_NAME}Config" DESTINATION share/6w485Lib/cmake)

# Expose public includes to other
# subprojects through cache variable.
set(${PROJECT_NAME}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)

# This makes the project importable from the build directory
export(TARGETS ${PROJECT_NAME} FILE "${PROJECT_NAME}Config.cmake")

### Unit test ###

# Every library has unit tests, of course
add_executable(hidapi_libtest
    test/testlib.cpp)

target_link_libraries(hidapi_libtest 
	${PROJECT_NAME}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# does the usage message work?
add_test (hidapi_libtest hidapi_libtest )

# add the install targets
install (TARGETS hidapi_libtest DESTINATION "test")

