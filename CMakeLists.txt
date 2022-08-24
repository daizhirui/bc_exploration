cmake_minimum_required(VERSION 3.16)
project(bc_exploration)

if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -fPIC ")
    set(CMAKE_CXX_FLAGS_DEBUG "-g")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
endif ()

if (NOT OpenMP_FOUND)
    find_package(OpenMP REQUIRED)
endif ()

if (NOT Python_FOUND)
    find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)
    include_directories(SYSTEM ${Python3_INCLUDE_DIRS})
    include_directories(SYSTEM ${Python3_NumPy_INCLUDE_DIRS})
endif ()

include_directories(SYSTEM deps/eigen)
add_subdirectory(deps/pybind11)
include_directories(SYSTEM deps/pybind11/include)
include_directories(${PROJECT_SOURCE_DIR}/bc_exploration/cpp/include)

set(SOURCE_FILES
        bc_exploration/cpp/src/python.cpp
        bc_exploration/cpp/src/exploration/astar.cpp
        bc_exploration/cpp/src/exploration/collision.cpp)

#add_library(bc_exploration STATIC ${PROJECT_SOURCE_DIR} ${SOURCE_FILES} bc_exploration/cpp/src/exploration/python.cpp)
#target_link_libraries(bc_exploration ${PYTHON_LIBRARIES} OpenMP::OpenMP_CXX)
#set_target_properties(exploration_cpp PROPERTIES SUFFIX ".so" PREFIX "_")

pybind11_add_module(_exploration_cpp ${SOURCE_FILES} NO_EXTRAS)
target_link_libraries(_exploration_cpp PUBLIC ${PYTHON_LIBRARIES} OpenMP::OpenMP_CXX)
