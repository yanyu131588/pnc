cmake_minimum_required(VERSION 3.10)

project(navigation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(INSTALL_BIN_DIR ${CMAKE_SOURCE_DIR}/bin)
set(INSTALL_LIB_DIR ${CMAKE_SOURCE_DIR}/lib)
set(INSTALL_INCLUDE_DIR ${INSTALL_LIB_DIR}/include)


find_package(yaml-cpp REQUIRED)
if(NOT yaml-cpp_FOUND)
    message(FATAL_ERROR "yaml-cpp not found!")
endif()

find_package(Python REQUIRED COMPONENTS Interpreter Development)
find_package(Eigen3 3.4.0 REQUIRED EXACT)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/utils/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rd)

include_directories(

  ${EIGEN3_INCLUDE_DIR}
)

set(Python_DIRS "/usr/include/python3.10")
include_directories(
  ${Python_DIRS}

)
add_subdirectory(3rd/json)

add_subdirectory(src/utils)
add_subdirectory(src/costmap)
add_subdirectory(src/pathPlanner)
add_subdirectory(src/localPlan)
add_subdirectory(src/loader)
add_subdirectory(src/comm)

add_subdirectory(examples/app)




