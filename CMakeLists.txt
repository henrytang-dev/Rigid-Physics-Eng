cmake_minimum_required(VERSION 3.5.0)
project(2d-constraint-engine VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

add_library(2d-constraint-engine 2d-constraint-engine.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
