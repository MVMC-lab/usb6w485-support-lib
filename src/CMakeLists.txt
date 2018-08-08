include(CTest)

include(ExternalProject)

add_subdirectory(comm6w485)
add_subdirectory(hidapi)
add_subdirectory(serial)
#add_subdirectory(test)

message("PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
