cmake_minimum_required(VERSION 3.10)

project(fem3d LANGUAGES CXX)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/modules/")

include (ExternalProject)
set(EIGEN_INCLUDE_DIR "${CMAKE_BINARY_DIR}/ext/eigen")
ExternalProject_Add(eigen
    GIT_REPOSITORY https://github.com/eigenteam/eigen-git-mirror.git
    GIT_TAG "3.3.7"
    GIT_PROGRESS TRUE
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND
        ${CMAKE_COMMAND} -E copy_directory
        <SOURCE_DIR>
        ${EIGEN_INCLUDE_DIR}
)

add_subdirectory(src/)
