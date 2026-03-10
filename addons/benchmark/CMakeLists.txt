cmake_minimum_required(VERSION 3.10)

project(hyrodyn_benchmark LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PROFILING_FLAGS -pg)

add_executable(hyrodyn_comp_perf benchmark.cpp)

target_compile_options(hyrodyn_comp_perf PRIVATE ${PROFILING_FLAGS})
target_link_options(hyrodyn_comp_perf PRIVATE ${PROFILING_FLAGS})

target_include_directories(hyrodyn_comp_perf PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(hyrodyn_comp_perf hyrodyn)
