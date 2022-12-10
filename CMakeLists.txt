cmake_minimum_required(VERSION 3.16)
project(bc_exploration)

if (NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DNDEBUG)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -fPIC ")
    set(CMAKE_CXX_FLAGS_DEBUG "-g")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
endif ()

if (NOT OpenMP_FOUND)
    find_package(OpenMP REQUIRED)
endif ()

if (NOT Python_FOUND)
    find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
endif ()

if (NOT DEFINED pybind11_add_module)
    add_subdirectory(deps/pybind11)
endif()

include_directories(SYSTEM deps/eigen)
include_directories(${PROJECT_SOURCE_DIR}/bc_exploration/cpp/include)

file(GLOB_RECURSE SOURCE_FILES bc_exploration/cpp/src/*.cpp)

pybind11_add_module(_exploration_cpp ${SOURCE_FILES} NO_EXTRAS)
target_link_libraries(_exploration_cpp PUBLIC ${PYTHON_LIBRARIES} pthread OpenMP::OpenMP_CXX)
target_include_directories(_exploration_cpp SYSTEM PUBLIC ${Python3_INCLUDE_DIRS} ${pybind11_INCLUDE_DIR})
