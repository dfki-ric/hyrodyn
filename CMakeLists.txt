cmake_minimum_required(VERSION 3.5)
project(hyrodyn VERSION 0.1 LANGUAGES CXX)
set(TARGET_NAME hyrodyn)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_PYTHON_BINDINGS "Build Python bindings using pybind11" ON)
option(RBDL_PKG_CONFIG_PATH "Custom path to RBDL's PkgConfig directory" "")

# Find RBDL and addons
find_package(PkgConfig REQUIRED)
if(RBDL_PKG_CONFIG_PATH)
    set(ENV{PKG_CONFIG_PATH} "${RBDL_PKG_CONFIG_PATH}:$ENV{PKG_CONFIG_PATH}")
endif()
pkg_search_module(rbdl REQUIRED IMPORTED_TARGET rbdl)
pkg_search_module(rbdl-urdfreader REQUIRED IMPORTED_TARGET rbdl-urdfreader)
list(APPEND PKGCONFIG_REQUIRES rbdl)
list(APPEND PKGCONFIG_REQUIRES rbdl-urdfreader)
string (REPLACE ";" " " PKGCONFIG_REQUIRES "${PKGCONFIG_REQUIRES}")

# Find Eigen3 and yaml-cpp
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Sources and headers
file(GLOB SOURCES "src/*.cpp")
file(GLOB HEADERS "src/*.hpp")


add_library(hyrodyn SHARED ${SOURCES} ${HEADERS})
set_target_properties(hyrodyn PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Include directories (RBDL install path)
target_include_directories(hyrodyn PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    # ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIR}
    ${RBDL_INSTALL_DIR}/include
)

# Link against installed RBDL libraries
target_link_libraries(hyrodyn
    PkgConfig::rbdl
    PkgConfig::rbdl-urdfreader
    yaml-cpp
)
add_executable(hyrodyn_bin ${CMAKE_CURRENT_SOURCE_DIR}/src/Main.cpp)
target_link_libraries(hyrodyn_bin hyrodyn)

if(BUILD_PYTHON_BINDINGS)
    find_package(pybind11 QUIET)
    target_include_directories(hyrodyn PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
    if(NOT pybind11_FOUND)
        message(WARNING "pybind11 not found, Python bindings will not be built.")
    else()
        message(STATUS "pybind11 found, building Python bindings.")
        set(PYBINDINGS_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/python_bindings)
        add_subdirectory(python)
    endif()
endif()


# Installation

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}.pc.in ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc DESTINATION lib/pkgconfig)
  
include(GNUInstallDirs)

# Install library and executable
install(TARGETS hyrodyn hyrodyn_bin
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# Install headers
install(FILES ${HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hyrodyn)
