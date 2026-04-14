cmake_minimum_required(VERSION 3.10)

# Project properties
set(PROJECT_NAMESPACE gepetto)
set(PROJECT_NAME ros-qualisys)
set(PROJECT_DESCRIPTION
    "ROS wrapper around the https://github.com/qualisys/qualisys_cpp_sdk package. See README.md."
)
set(PROJECT_URL https://github.com/${PROJECT_NAMESPACE}/${PROJECT_NAME})

# Project options
option(SUFFIX_SO_VERSION "Suffix library name with its version" ON)

# Project configuration
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CUSTOM_HEADER_DIR ${PROJECT_NAME})

# Check if the submodule cmake have been initialized
set(JRL_CMAKE_MODULES "${CMAKE_CURRENT_LIST_DIR}/cmake")
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/cmake/base.cmake")
  if(${CMAKE_VERSION} VERSION_LESS "3.14.0")
    message(
      FATAL_ERROR
        "\nPlease run the following command first:\ngit submodule update --init\n"
    )
  else()
    include(FetchContent)
    FetchContent_Declare(
      "jrl-cmakemodules"
      GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git")
    FetchContent_MakeAvailable("jrl-cmakemodules")
    FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
  endif()
endif()

# JRL-cmakemodule setup
include("${JRL_CMAKE_MODULES}/base.cmake")
include("${JRL_CMAKE_MODULES}/boost.cmake")

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

# Define the README file as the main page.
set(DOXYGEN_INPUT "${DOXYGEN_INPUT} \"${PROJECT_SOURCE_DIR}/README.md\" \"${PROJECT_SOURCE_DIR}/package.xml\"" CACHE STRING " ")
set(DOXYGEN_USE_MDFILE_AS_MAINPAGE "\"${PROJECT_SOURCE_DIR}/README.md\"" CACHE STRING " ")

# Project dependencies
find_package(qualisys_cpp_sdk REQUIRED)
find_package(roscpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS unit_test_framework)

# Main Library
set(${PROJECT_NAME}_HEADERS include/${PROJECT_NAME}/qualisys-to-ros.hpp)

set(${PROJECT_NAME}_SOURCES src/qualisys-to-ros.cpp)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
                                   ${${PROJECT_NAME}_HEADERS})
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<INSTALL_INTERFACE:include> ${roscpp_INCLUDE_DIRS}
         ${tf2_INCLUDE_DIRS} ${tf2_ros_INCLUDE_DIRS}
         ${geometry_msgs_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC ${roscpp_LIBRARIES} ${tf2_LIBRARIES} ${tf2_ros_LIBRARIES}
         ${geometry_msgs_LIBRARIES} qualisys_cpp_sdk)
if(SUFFIX_SO_VERSION)
  set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
endif()

# Main Executable
add_executable(${PROJECT_NAME}-node src/ros-qualisys.cpp)
target_link_libraries(${PROJECT_NAME}-node ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}-node PROPERTIES OUTPUT_NAME
                                                      "${PROJECT_NAME}")
set_target_properties(${PROJECT_NAME}-node PROPERTIES INSTALL_RPATH
                                                      "\$ORIGIN/../lib")

# Unit tests
add_subdirectory(tests)

# Installation
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)
install(TARGETS ${PROJECT_NAME}-node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
