cmake_minimum_required(VERSION 3.10)

# ---------------- Dependencies ----------------

# GoogleTest
find_package(GTest REQUIRED)

# Eigen is header-only but needed for include dirs
find_package(Eigen3 REQUIRED)

# ---------------- Test Target ----------------

add_executable(hyrodyn_tests
    test_hyrodyn.cpp
)

target_compile_features(hyrodyn_tests PRIVATE cxx_std_17)

target_include_directories(hyrodyn_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(hyrodyn_tests
    hyrodyn
    GTest::gtest
    GTest::gtest_main
    pthread
)

# ---------------- Register Tests ----------------

add_test(
    NAME hyrodyn_tests
    COMMAND hyrodyn_tests
)
set_tests_properties(hyrodyn_tests PROPERTIES
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)