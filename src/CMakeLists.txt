cmake_minimum_required (VERSION 2.8)

project ("RayTracer")

message("Building ${CMAKE_BUILD_TYPE}")

#DEBUG flags
set(CMAKE_CXX_FLAGS_DEBUG "-O2 -g -Wall")
#RELEASE flags
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "-s -O3")

if(DEFINED NTHREADS)
    if ((${NTHREADS} GREATER 0) AND (${NTHREADS} LESS 11))
        add_definitions( -DNTHREADS=${NTHREADS} )
        message("Using custom number of threads: ${NTHREADS}.")
    else()
        message("Using default number of threads: 4.")
    endif()     
endif()

#Lookup directories for header files
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/random
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/easy_bmp
    /usr/local/include
)

#link_directories (
#    ${CMAKE_CURRENT_SOURCE_DIR}/libs/random 
#    /mnt/DEV/HUJI/ray_tracer/bimage
#    /usr/local/lib/OpenMesh
#)

#Dependencies - 3rd party libraries
find_library (LIB_OPENMESH OpenMeshCore /usr/local/lib/OpenMesh)
find_library (LIB_PTHREAD pthread)
#Included dependencies
find_library (LIB_RANDOM randomaelf64 ${CMAKE_CURRENT_SOURCE_DIR}/libs/random)
set (SRC_BMP ${CMAKE_CURRENT_SOURCE_DIR}/libs/easy_bmp/EasyBMP.cpp)

#The RayTrace library

set(LIBRARY_NAME "RayTrace")
aux_source_directory (${CMAKE_CURRENT_SOURCE_DIR} SRC_FILES)
add_library(
    ${LIBRARY_NAME}
    STATIC
    ${SRC_FILES}
    ${SRC_BMP}
)

target_link_libraries(
    ${LIBRARY_NAME}
    ${LIB_OPENMESH}
    ${LIB_PTHREAD}
    ${LIB_RANDOM}
)

#Executable with example scene

set(EXECUTABLE_NAME "example")

add_executable (
    ${EXECUTABLE_NAME} 
    ${CMAKE_CURRENT_SOURCE_DIR}/example/MyScene.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/example/Arguments.cc
)

target_link_libraries(
    ${EXECUTABLE_NAME}
    ${LIBRARY_NAME}
)
