cmake_minimum_required(VERSION 3.0.2)
project(irisDemo)

set(CMAKE_BUILD_TYPE Debug)

set(PROJECT_ROOT_DIR ${CMAKE_SOURCE_DIR})
set(IRIS_INCLUDE_DIR ${PROJECT_ROOT_DIR}/include/iris)
set(IRIS_LIB_DIR ${PROJECT_ROOT_DIR}/lib/iris)
set(MOSEK_DIR ${PROJECT_ROOT_DIR}/../mosek/7/tools/platform/linux64x86)

set(CMAKE_EXE_LINKER_FLAGS "-Wl,-rpath,${IRIS_LIB_DIR}")

include_directories(${IRIS_INCLUDE_DIR})
include_directories(${IRIS_INCLUDE_DIR}/iris)
include_directories(${IRIS_INCLUDE_DIR}/eigen3)
include_directories(/usr/include/python3.8)

find_library(IRIS_LIB iris HINTS ${IRIS_LIB_DIR})
find_library(IRIS_GEOMETRY_LIB iris_geometry HINTS ${IRIS_LIB_DIR})
find_library(IRIS_CVXGEN_LDP_CPP_LIB NAMES iris_cvxgen_ldp_cpp HINTS ${IRIS_LIB_DIR})
find_library(IRIS_CVXGEN_LDP_LIB iris_cvxgen_ldp HINTS ${IRIS_LIB_DIR})
find_library(MOSEK_LIB NAMES mosek64 HINTS ${MOSEK_DIR}/bin)

add_executable(irisDemo ./src/iris_demo.cpp)

target_link_libraries(irisDemo
    ${IRIS_LIB}
    ${IRIS_GEOMETRY_LIB}
    ${IRIS_CVXGEN_LDP_CPP_LIB}
    ${MOSEK_LIB}
    /usr/lib/python3.8/config-3.8-x86_64-linux-gnu/libpython3.8.so
)

target_include_directories(irisDemo PUBLIC ${IRIS_LIB_DIR})
target_include_directories(irisDemo PUBLIC ${MOSEK_DIR}/bin)
