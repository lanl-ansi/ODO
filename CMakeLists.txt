cmake_minimum_required(VERSION 3.2)

project(ODO)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

# Include Armadillo
execute_process(
  COMMAND ${CMAKE_COMMAND} -E tar xzf ${PROJECT_SOURCE_DIR}/ext_lib/armadillo.tar.xz
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/ext_lib
  )
find_package(ARMADILLO)
include_directories(${ARMADILLO_ROOT_DIR})
set(LIBS ${LIBS} armadillo)
add_subdirectory(${ARMADILLO_ROOT_DIR})

# Debug mode
option(debug "Turn on debug mode" OFF)

# Find IPOPT (optional)
option(Ipopt "Link to IPOPT libraries" ON)

# Find XLNT (optional)
option(Xlnt "Link to XLNT libraries" ON)

# Find BONMIN (optional)
option(Bonmin "Link to BONMIN libraries" OFF)

# Find CPLEX (optional)
option(Cplex "Link to CPLEX libraries" OFF)


# Find Ipopt.
if (Ipopt)
	message(STATUS "Enable IPOPT")
	add_definitions(-DUSE_IPOPT)
	find_package(IPOPT REQUIRED)
	include_directories(${IPOPT_INCLUDE_DIRS}/..)
	set(LIBS ${LIBS} ${IPOPT_LIBRARIES})
endif()

# Find xlnt
if (Xlnt)
	message(STATUS "Enable XLNT")
	add_definitions(-DUSE_XLNT)
	find_package(XLNT REQUIRED)
	include_directories(${XLNT_INCLUDE_DIRS}/..)
	set(LIBS ${LIBS} ${XLNT_LIBRARIES})
endif()



# Find BONMIN?
if (Bonmin)
    message(STATUS "Enable BONMIN")
    add_definitions(-DUSE_BONMIN)
    find_package(BONMIN)
    include_directories(${BONMIN_INCLUDE_DIRS}/..)
    set(LIBS ${LIBS} ${BONMIN_LIBRARIES})
endif()

# Find CPLEX?
if (Cplex)
    message(STATUS "Enable CPLEX")
    add_definitions(-DUSE_CPLEX)
    find_package(CPLEX)
    include_directories(${CPLEX_INCLUDE_DIRS})
    #link_directories(${CPLEX_LINKS})
    set(LIBS ${LIBS} ${CPLEX_LIBRARIES})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -DNDEBUG -DIL_STD")
endif()
# Include cpp_option_parser
include_directories(${PROJECT_SOURCE_DIR}/ext_lib/cpp_option_parser)
add_subdirectory(ext_lib/cpp_option_parser)
set(LIBS ${LIBS} option_parser)
# Include rapidjson
include_directories(${PROJECT_SOURCE_DIR}/ext_lib/rapidjson-1.1.0/include)
# Include Power dirs
include_directories(${PROJECT_SOURCE_DIR}/examples/MINLP/Power)

set(LIBS ${LIBS} ${PROJECT_SOURCE_DIR}/lib ${PROJECT_SOURCE_DIR}/ext_lib/Ipopt/lib ${PROJECT_SOURCE_DIR}/lib/Debug)
# Compiler options
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Ofast")

# Build options
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -lpthread")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath,'\$ORIGIN/ext_lib/Ipopt/lib'")
add_subdirectory(src)
add_subdirectory(examples)

#if (debug)
#set(CMAKE_BUILD_TYPE "Debug")
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/Debug)
#endif()




#Get user defined configuration including passing project source directory into c++ code
configure_file(GravityConfig.h.in ${PROJECT_SOURCE_DIR}/include/gravity/GravityConfig.h)
