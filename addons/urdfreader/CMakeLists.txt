cmake_minimum_required(VERSION 3.10)
project(urdfreader)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find installed RBDL
find_package(RBDL REQUIRED)
find_package(Eigen3 REQUIRED)

add_subdirectory(thirdparty)

# Library
add_library(urdfreader urdfreader.cc transmission_parser.cc)

target_include_directories(
  urdfreader
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/urdfparser/include>
    $<INSTALL_INTERFACE:include> ${RBDL_INCLUDE_DIR})
target_link_libraries(urdfreader PUBLIC ${RBDL_LIBRARIES} Eigen3::Eigen
                                        urdfparser)

# Install library
install(
  TARGETS urdfreader
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install headers
file(GLOB headers "*.h")

install(FILES ${headers} DESTINATION include/urdfreader)
