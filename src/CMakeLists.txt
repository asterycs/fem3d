cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} \
    -Wall \
    -Wextra \
    -Werror \
    -Wno-error=unused-parameter \
    -Wconversion \
    -Wno-error=conversion \
    -fasynchronous-unwind-tables \
    -fstack-protector-all \
    -Werror=format-security \
    -Werror=implicit-function-declaration \
    -Wdisabled-optimization \
    -Wformat \
    -Wcast-align \
    -Wformat-security \
    -Wmissing-braces \
    -Wparentheses \
    -Wpointer-arith \
    -Wredundant-decls \
    -Wreturn-type \
    -Wsign-compare \
    -Wuninitialized \
    -Wunreachable-code \
    -Wunused \
    -Wunused-label \
    -Wunused-value \
    -Wunused-variable")

find_package(Magnum REQUIRED
    GL
    MeshTools
    Shaders
    SceneGraph
    Trade
    Sdl2Application
    Primitives)

find_package(MagnumPlugins REQUIRED
        StbTrueTypeFont)
    
find_package(MagnumExtras REQUIRED
    Ui)

find_package(OpenMP)
if(OPENMP_FOUND)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

corrade_add_resource(fem3d_RESOURCES resources.conf)

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)

include_directories(${MAGNUM_INCLUDE_DIR})
include_directories(${MAGNUMEXTRAS_INCLUDE_DIR})
include_directories(${CORRADE_INCLUDE_DIR})
include_directories(SYSTEM ${EIGEN_INCLUDE_DIR})

file(GLOB fem3d_SRC *.cpp)

add_executable(fem3d ${fem3d_SRC} ${fem3d_RESOURCES})

add_dependencies(fem3d eigen)
target_link_libraries(fem3d PRIVATE
    Magnum::Application
    Magnum::GL
    Magnum::Magnum
    Magnum::MeshTools
    Magnum::SceneGraph
    Magnum::Shaders
    Magnum::Trade
    Magnum::Primitives
    MagnumExtras::Ui
    MagnumPlugins::StbTrueTypeFont)

if(CORRADE_TARGET_EMSCRIPTEN)
    install(TARGETS fem3d DESTINATION ${MAGNUM_DEPLOY_PREFIX})
    install(FILES
        index.html
        EmscriptenApplication.js
        fem3dstyle.css
        DESTINATION ${MAGNUM_DEPLOY_PREFIX})
    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/fem3d.js.mem
        ${CMAKE_CURRENT_BINARY_DIR}/fem3d.wasm
        DESTINATION ${MAGNUM_DEPLOY_PREFIX} OPTIONAL)
endif()
