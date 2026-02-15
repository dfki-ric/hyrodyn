cmake_minimum_required(VERSION 3.10)

project(
  hyrodyn
  VERSION 1.0
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

option(BUILD_PYTHON_BINDINGS "Build Python bindings using pybind11" OFF)
option(RBDL_PKG_CONFIG_PATH "Custom path to RBDL's PkgConfig directory" "")

# ---------------- Dependencies ----------------
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)

if(RBDL_PKG_CONFIG_PATH)
  set(ENV{PKG_CONFIG_PATH} "${RBDL_PKG_CONFIG_PATH}:$ENV{PKG_CONFIG_PATH}")
endif()
pkg_search_module(rbdl REQUIRED IMPORTED_TARGET rbdl)
list(APPEND PKGCONFIG_REQUIRES rbdl)

# Internal urdfreader
add_subdirectory(addons/urdfreader)
message(STATUS "RBDL_LIBRARIES = ${RBDL_LIBRARIES}")
message(STATUS "RBDL_INCLUDE_DIR = ${RBDL_INCLUDE_DIR}")

# ---------------- Sources ----------------

file(GLOB SOURCES "src/*.cpp")
file(GLOB HEADERS "src/*.hpp")

# ---------------- Library ----------------

add_library(hyrodyn SHARED ${SOURCES})

set_target_properties(hyrodyn PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(hyrodyn PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
                                          ${RBDL_INCLUDE_DIR}/include)

target_link_libraries(hyrodyn PkgConfig::rbdl urdfreader yaml-cpp)
add_executable(hyrodyn_bin ${CMAKE_CURRENT_SOURCE_DIR}/src/Main.cpp)
target_link_libraries(hyrodyn_bin hyrodyn)

# ---------------- Python Bindings ----------------

if(BUILD_PYTHON_BINDINGS)
  find_package(pybind11 QUIET)

  if(pybind11_FOUND)
    add_subdirectory(python)
  else()
    message(WARNING "pybind11 not found, Python bindings disabled.")
  endif()
endif()

# ---------------- Installation ----------------

include(GNUInstallDirs)

install(
  TARGETS hyrodyn hyrodyn_bin
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(FILES ${HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hyrodyn)


# cmake ..   -DRBDL_PKG_CONFIG_PATH=/home/dfki.uni-bremen.de/rkumar/dynamics/install/lib/pkgconfig -DCMAKE_PREFIX_PATH=/home/dfki.uni-bremen.de/rkumar/dynamics/install
