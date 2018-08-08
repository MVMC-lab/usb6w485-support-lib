#Define library. Only source file here!
project(Comm6W485 LANGUAGES CXX)

# does this system provide specific functions?
include (${CMAKE_ROOT}/Modules/CheckFunctionExists.cmake)

# Check for system, is provide ASA485_init function
check_function_exists (Comm6W485_init HAVE_COMM_INIT)

add_library(${PROJECT_NAME}
	src/comm6w485.cpp
)

target_link_libraries(${PROJECT_NAME}
	HidAPI
    serial
)

add_dependencies(${PROJECT_NAME} HidAPI)

set(LIBS HidAPI)

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
message("ASA485 include : ${serial_INCLUDE_DIRS} ${ASAComm_INCLUDE_DIRS}")
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${HidAPI_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${serial_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# If we have compiler requirements for this library, list them
# here
target_compile_features(${PROJECT_NAME}
    PUBLIC cxx_auto_type
    PRIVATE cxx_variadic_templates)

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS ${PROJECT_NAME}
  EXPORT "${PROJECT_NAME}Config"
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

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

include_directories(
	${HidAPI_INCLUDE_DIRS}
	${serial_INCLUDE_DIRS}
)


### Unit test ###

# Every library has unit tests, of course
add_executable(Comm6W485_libtest
    test/testlib.cpp)

target_link_libraries(Comm6W485_libtest 
	${PROJECT_NAME}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${HidAPI_INCLUDE_DIRS}>
	$<BUILD_INTERFACE:${serial_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# does the usage message work?
add_test (Comm6W485_libtest Comm6W485_libtest )

# add the install targets
install (TARGETS Comm6W485_libtest DESTINATION "test")

