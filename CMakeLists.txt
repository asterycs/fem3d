cmake_minimum_required(VERSION 3.10)

project(fem3d)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/modules/")

add_subdirectory(src/)
