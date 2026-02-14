# Python Bindings Module CMake File

set(PYBIND_SOURCES
    src/hyrodyn_limit_checker_py.cpp
    src/hyrodyn_py.cpp
    src/utils_py.cpp
)

foreach(pybind_src ${PYBIND_SOURCES})
    get_filename_component(modulename ${pybind_src} NAME_WE)

    pybind11_add_module(${modulename} MODULE ${pybind_src})

    target_link_libraries(${modulename}
        PRIVATE
            hyrodyn
            Eigen3::Eigen
            yaml-cpp
    )

    target_include_directories(${modulename}
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/..
    )

    set_target_properties(${modulename} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${PYBINDINGS_OUTPUT_DIR}
    )
endforeach()
