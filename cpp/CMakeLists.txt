cmake_minimum_required(VERSION 3.16)
project(cjdemo)

# Set build type
set(CMAKE_BUILD_TYPE "Release")

# Use appropriate flags for MSVC
if (MSVC)
  add_definitions(/W4 /O2)
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2 -std=c++11")
endif()

# Project version
set(${PROJECT_NAME}_VERSION_MAJOR 1)
set(${PROJECT_NAME}_VERSION_MINOR 0)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)

# Find CGAL
find_package(CGAL QUIET COMPONENTS)
if (CGAL_FOUND)
  message(STATUS "CGAL found")
  message(STATUS ${CGAL_LIBRARIES})
  message(STATUS ${CGAL_3RD_PARTY_LIBRARIES})
else()
  message(SEND_ERROR "CGAL library is required")
  return()
endif()

# Find Eigen3
find_package(Eigen3 3.1.0 QUIET)
include(CGAL_Eigen3_support)

# Source files
file(GLOB SRC_FILES src/*.cpp)
add_executable(${PROJECT_NAME} ${SRC_FILES})

# Link libraries
target_link_libraries(${PROJECT_NAME} CGAL::CGAL)
