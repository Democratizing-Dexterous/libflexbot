cmake_minimum_required(VERSION 3.16)

project(libflexbot LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)
set(PYTHON_ABI "${Python3_VERSION_MAJOR}${Python3_VERSION_MINOR}")
find_package(Boost REQUIRED COMPONENTS python${PYTHON_ABI} numpy${PYTHON_ABI})
find_package(yaml-cpp REQUIRED)

add_library(libflexbot_core STATIC
    src/canfd.cpp
    src/robot.cpp
)

target_include_directories(libflexbot_core PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)

target_compile_definitions(libflexbot_core PUBLIC
    LIBFLEXBOT_PROJECT_ROOT="${PROJECT_SOURCE_DIR}"
)

target_link_libraries(libflexbot_core PUBLIC
    dl
    pthread
    yaml-cpp
)

set_target_properties(libflexbot_core PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

Python3_add_library(_libflexbot MODULE
    src/python_bindings.cpp
)

target_include_directories(_libflexbot PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${Python3_NumPy_INCLUDE_DIRS}
)

target_link_libraries(_libflexbot PRIVATE
    libflexbot_core
    Boost::python${PYTHON_ABI}
    Boost::numpy${PYTHON_ABI}
    Python3::Python
)

set_target_properties(_libflexbot PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python/libflexbot
    PREFIX ""
)

add_custom_command(TARGET _libflexbot POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${PROJECT_SOURCE_DIR}/libs/libcontrolcanfd.so
        ${CMAKE_BINARY_DIR}/python/libflexbot/libcontrolcanfd.so
)
