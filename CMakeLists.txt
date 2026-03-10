cmake_minimum_required(VERSION 3.10)

project(
  hyrodyn
  VERSION 1.0
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

option(BUILD_PYTHON_BINDINGS "Build Python bindings using pybind11" ON)
option(BUILD_TESTING "Build unit tests" ON)
option(BUILD_ADDON_BENCHMARK "Build the benchmarking tool" ON)
# ---------------- Dependencies ----------------
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# RBDL minimal build configuration set(RBDL_BUILD_STATIC ON CACHE BOOL "" FORCE)
# set(RBDL_BUILD_ADDON_URDFREADER OFF CACHE BOOL "" FORCE)
# set(RBDL_BUILD_ADDON_GEOMETRY OFF CACHE BOOL "" FORCE)
# set(RBDL_BUILD_ADDON_LUAMODEL OFF CACHE BOOL "" FORCE)
# set(RBDL_BUILD_PYTHON_WRAPPER OFF CACHE BOOL "" FORCE) set(RBDL_BUILD_TESTS
# OFF CACHE BOOL "" FORCE) set(RBDL_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

add_subdirectory(external/rbdl)
target_include_directories(
  rbdl PUBLIC $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/external/rbdl/include>)

# Internal urdfreader
add_subdirectory(addons/urdfreader)

# ---------------- Sources ----------------

file(GLOB SOURCES "src/*.cpp")
file(GLOB HEADERS "src/*.hpp")

# ---------------- Library ----------------

add_library(hyrodyn SHARED ${SOURCES})
set_target_properties(hyrodyn PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(hyrodyn PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(hyrodyn rbdl urdfreader yaml-cpp)
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

include(CTest)
if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

if(BUILD_ADDON_BENCHMARK)
  add_subdirectory(addons/benchmark)
endif()
# ---------------- Installation ----------------

include(GNUInstallDirs)

install(
  TARGETS hyrodyn hyrodyn_bin
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(FILES ${HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hyrodyn)
