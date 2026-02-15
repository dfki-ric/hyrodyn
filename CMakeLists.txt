cmake_minimum_required(VERSION 3.10)

project(
  hyrodyn
  VERSION 1.0
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

option(BUILD_PYTHON_BINDINGS "Build Python bindings using pybind11" OFF)

# ---------------- Dependencies ----------------

find_package(RBDL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Internal urdfreader
add_subdirectory(addons/urdfreader)

# ---------------- Sources ----------------

file(GLOB SOURCES "src/*.cpp")
file(GLOB HEADERS "src/*.hpp")

# ---------------- Library ----------------

add_library(hyrodyn SHARED ${SOURCES})

set_target_properties(hyrodyn PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(
  hyrodyn PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${RBDL_INCLUDE_DIR}
                 ${RBDL_INCLUDE_DIRS})

target_link_libraries(hyrodyn PUBLIC ${RBDL_LIBRARIES} urdfreader Eigen3::Eigen
                                     yaml-cpp)

# ---------------- Executable ----------------

add_executable(hyrodyn_bin src/Main.cpp)

# IMPORTANT: must link RBDL explicitly because it is not an imported target
target_link_libraries(hyrodyn_bin PRIVATE hyrodyn ${RBDL_LIBRARIES})

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
