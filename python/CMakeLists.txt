# ---- Python bindings for hyrodyn ----

set(PYBIND_SOURCES
    src/hyrodyn_py.cpp
    src/hyrodyn_limit_checker_py.cpp
    src/utils_py.cpp
)

# Internal CMake target name (must differ from library)
pybind11_add_module(hyrodyn_python MODULE ${PYBIND_SOURCES})

target_link_libraries(hyrodyn_python
    PRIVATE
        hyrodyn
        Eigen3::Eigen
        yaml-cpp
)

target_include_directories(hyrodyn_python
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
)

# Python-visible module name
set_target_properties(hyrodyn_python PROPERTIES
    OUTPUT_NAME hyrodyn
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python_bindings
)

