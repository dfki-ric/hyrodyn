cmake_minimum_required(VERSION 3.10)
project(hyrodyn VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

option(BUILD_PYTHON_BINDINGS "Build Python bindings using pybind11" OFF)
option(RBDL_PKG_CONFIG_PATH "Custom path to RBDL's PkgConfig directory" "")

# # # Find RBDL and addons
# find_package(PkgConfig REQUIRED)
# if(RBDL_PKG_CONFIG_PATH)
#     set(ENV{PKG_CONFIG_PATH} "${RBDL_PKG_CONFIG_PATH}:$ENV{PKG_CONFIG_PATH}")
# endif()
# pkg_search_module(rbdl REQUIRED IMPORTED_TARGET rbdl)
# list(APPEND PKGCONFIG_REQUIRES rbdl)

# ---- Dependencies ----

find_package(RBDL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_subdirectory(addons/urdfreader)

# ---- Sources ----

file(GLOB SOURCES "src/*.cpp")
file(GLOB HEADERS "src/*.hpp")

add_library(hyrodyn SHARED ${SOURCES})
set_target_properties(hyrodyn PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(hyrodyn
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${RBDL_INCLUDE_DIR}
)

target_link_libraries(hyrodyn
    PUBLIC
        ${RBDL_LIBRARIES}
        urdfreader
        Eigen3::Eigen
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

# CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}.pc.in ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc @ONLY)
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
